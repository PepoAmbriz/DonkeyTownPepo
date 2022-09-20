import cv2
import numpy as np
h = 179
w = 239 
lw = 24
a = 32
b = 45
c = 57 
th = 1
H = 400 
W = 700
xo = int(W-w)/2
yo = int(H-h)/2
def draw_contour(img,ll,wd,col): 
	cv2.line(img,(wd+xo,ll+yo),(w-wd-1+xo,ll+yo),col,th)
	cv2.line(img,(w-wd-1+xo,ll+yo),(w-1-ll+xo,wd+yo),col,th)
	cv2.line(img,(w-1-ll+xo,wd+yo),(w-1-ll+xo,h-wd+yo),col,th)
	cv2.line(img,(w-1-ll+xo,h-wd+yo),(w-wd+xo,h-ll-1+yo),col,th)
	cv2.line(img,(w-wd+xo,h-ll-1+yo),(wd+xo,h-ll-1+yo),col,th)
	cv2.line(img,(wd+xo,h-ll-1+yo),(ll+xo,h-wd-1+yo),col,th)
	cv2.line(img,(ll+xo,h-wd-1+yo),(ll+xo,wd+yo),col,th)
	cv2.line(img,(ll+xo,wd+yo),(wd+xo,ll+yo),col,th)
	return img 

img_3 = np.zeros([H,W,3],dtype=np.uint8)
img_3.fill(0)
yellow = (0,255,255)
img_3 = draw_contour(img_3,0*24,a,yellow)
img_3 = draw_contour(img_3,1*24,b,yellow)
img_3 = draw_contour(img_3,2*24,c,yellow)



# or img[:] = 255
cv2.imshow('3 Channel Window', img_3)
print("image shape: ", img_3.shape)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("circuit.png", img_3)