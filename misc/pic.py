import cv2
import numpy as np

# Leeres weißes Bild
img = np.ones((800, 800, 3), dtype=np.uint8) * 255

# Rotes Rechteck (x1, y1) – (x2, y2)
cv2.rectangle(img, (50, 50), (750, 750), (0, 0, 255), 7)

# Senkrechter Strich durch die Mitte
center_x = img.shape[1] // 2
cv2.line(img, (center_x, 25), (center_x, 775), (0, 255, 0), 7)

# Anzeigen und speichern
cv2.imshow("Red Square with Line", img)
cv2.imwrite("red_square_with_line.png", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
