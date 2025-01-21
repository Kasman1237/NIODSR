import cv2
import numpy as np

def draw_rectangle(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: {x}, {y}")

window_name = "Test Window"
cv_image = np.zeros((512, 700, 3), np.uint8)

cv2.imshow(window_name, cv_image)
cv2.setMouseCallback(window_name, draw_rectangle)

while True:
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
