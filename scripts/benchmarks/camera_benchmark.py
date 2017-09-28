from rick.async import AsyncCamera
import cv2
import time

camera = AsyncCamera(1)

while True:

    ok, frame = camera.read()

    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break
    time.sleep(1)


