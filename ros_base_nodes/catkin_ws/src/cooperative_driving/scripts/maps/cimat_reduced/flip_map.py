import cv2

src = cv2.imread('./map.png')
fliped = cv2.flip(src,0) #0: vertical flip
cv2.imshow("image",fliped)
cv2.waitKey(0)
cv2.imwrite("map_flip.bmp",fliped)
