import cv2

# 1. Initialize video capture (0 is usually the default webcam)
cap = cv2.VideoCapture(4)

# 2. Get frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 20.0

# 3. Define codec and create VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
out = cv2.VideoWriter('output.mp4', fourcc, fps, (frame_width, frame_height))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 4. Write and display
    out.write(frame)
    cv2.imshow('Recording... Press Q to Stop', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 5. Release resources
cap.release()
out.release()
cv2.destroyAllWindows()
