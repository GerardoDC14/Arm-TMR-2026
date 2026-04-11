import time
from collections import namedtuple

try:
    import hid
except ImportError as exc:
    raise ImportError(
        "The Python 'hid' module is required for SpaceMouse support. "
        "Install hidapi/python-hid for this environment."
    ) from exc


VENDOR_ID = 0x046D
PRODUCT_ID = 0xC627

AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])
ButtonSpec = namedtuple("ButtonSpec", ["channel", "byte", "bit"])
SpaceNavigator = namedtuple(
    "SpaceNavigator", ["t", "x", "y", "z", "roll", "pitch", "yaw", "buttons"]
)


class ButtonState(list):
    def __int__(self):
        return sum((b << i) for (i, b) in enumerate(reversed(self)))


class DeviceSpec:
    def __init__(self, hid_id, mappings, button_mapping, axis_scale=350.0):
        self.hid_id = hid_id
        self.mappings = mappings
        self.button_mapping = button_mapping
        self.axis_scale = axis_scale
        self.device = None
        self.state = {
            "t": time.time(),
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "buttons": ButtonState([0] * len(button_mapping)),
        }

    def _set_nonblocking(self, enabled):
        if hasattr(self.device, "set_nonblocking"):
            self.device.set_nonblocking(enabled)
            return

        if hasattr(self.device, "nonblocking"):
            self.device.nonblocking = int(enabled)
            return

        raise AttributeError("The installed hid module does not support nonblocking reads")

    def open(self):
        try:
            if hasattr(hid, "device"):
                self.device = hid.device()
                self.device.open(self.hid_id[0], self.hid_id[1])
            elif hasattr(hid, "Device"):
                self.device = hid.Device(vid=self.hid_id[0], pid=self.hid_id[1])
            else:
                raise AttributeError("Unsupported hid module: expected hid.device or hid.Device")

            self._set_nonblocking(True)
        except Exception as exc:
            if self.device and hasattr(self.device, "close"):
                self.device.close()
            self.device = None

            hid_exception = getattr(hid, "HIDException", None)
            if hid_exception and isinstance(exc, hid_exception) and "Permission denied" in str(exc):
                raise PermissionError(
                    "Space Explorer was detected, but Linux denied access to /dev/hidraw*. "
                    "Add a udev rule for vendor 046d product c627 or adjust the device permissions."
                ) from exc

            raise

    def close(self):
        if self.device:
            self.device.close()
            self.device = None

    def read(self):
        if self.device is None:
            return None

        data = self.device.read(64)
        if not data:
            return None

        report_id = data[0]
        if report_id not in [1, 2, 3]:
            return None

        for name, spec in self.mappings.items():
            if spec.channel == report_id:
                raw = int.from_bytes(
                    [data[spec.byte1], data[spec.byte2]],
                    byteorder="little",
                    signed=True,
                )
                self.state[name] = spec.scale * raw / self.axis_scale

        for i, button in enumerate(self.button_mapping):
            if button.channel == report_id:
                mask = 1 << button.bit
                self.state["buttons"][i] = 1 if (data[button.byte] & mask) != 0 else 0

        self.state["t"] = time.time()
        return SpaceNavigator(**self.state)


device_spec = DeviceSpec(
    hid_id=(VENDOR_ID, PRODUCT_ID),
    mappings={
        "x": AxisSpec(1, 1, 2, 1),
        "y": AxisSpec(1, 3, 4, -1),
        "z": AxisSpec(1, 5, 6, -1),
        "pitch": AxisSpec(2, 1, 2, -1),
        "roll": AxisSpec(2, 3, 4, -1),
        "yaw": AxisSpec(2, 5, 6, 1),
    },
    button_mapping=[
        ButtonSpec(3, 1, 7),
        ButtonSpec(3, 1, 6),
        ButtonSpec(3, 1, 5),
        ButtonSpec(3, 1, 4),
        ButtonSpec(3, 1, 3),
        ButtonSpec(3, 1, 2),
        ButtonSpec(3, 1, 1),
        ButtonSpec(3, 1, 0),
        ButtonSpec(3, 2, 6),
        ButtonSpec(3, 2, 5),
        ButtonSpec(3, 2, 4),
        ButtonSpec(3, 2, 3),
        ButtonSpec(3, 2, 2),
        ButtonSpec(3, 2, 1),
        ButtonSpec(3, 2, 0),
    ],
)


def open():
    device_spec.open()
    return True


def close():
    device_spec.close()


def read():
    return device_spec.read()
