import spacenavigator_hidapi as spnav

try:
    spnav.open()
except PermissionError as exc:
    raise SystemExit(str(exc))

try:
    while True:
        state = spnav.read()
        if state:
            print(f"X: {state.x:.2f}, Y: {state.y:.2f}, Z: {state.z:.2f}, Roll: {state.roll:.2f}, Pitch: {state.pitch:.2f}, Yaw: {state.yaw:.2f}, Buttons: {state.buttons}")
except KeyboardInterrupt:
    pass
finally:
    spnav.close()
