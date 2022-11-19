import cv2
import numpy as np
h = 222
w = 222 
lw = 29
a = 30
b = 57
c = 84 
th = 1
H = 601 
W = 601
xo = int(W-w)/2+82
yo = int(H-h)/2+82
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
img_3 = draw_contour(img_3,0*lw,a,yellow)
img_3 = draw_contour(img_3,1*lw,b,yellow)
img_3 = draw_contour(img_3,2*lw,c,yellow)

xo = int(W-w)/2-81
yo = int(H-h)/2-81
img_3 = draw_contour(img_3,0*lw,a,yellow)
img_3 = draw_contour(img_3,1*lw,b,yellow)
img_3 = draw_contour(img_3,2*lw,c,yellow)

xo = int(W)/2
yo = int(H)/2
img_3[yo-55:yo+55,xo-55:xo+55] = (0,0,0)

cv2.line(img_3,(xo-55,yo-lw),(xo+55,yo-lw),yellow,th)
cv2.line(img_3,(xo-55,yo),(xo+55,yo),yellow,th)
cv2.line(img_3,(xo-55,yo+lw),(xo+55,yo+lw),yellow,th)

cv2.line(img_3,(xo-lw,yo+55),(xo-lw,yo-55),yellow,th)
cv2.line(img_3,(xo,yo+55),(xo,yo-55),yellow,th)
cv2.line(img_3,(xo+lw,yo+55),(xo+lw,yo-55),yellow,th)

img_3[yo-lw+1:yo+lw,xo-lw-10:xo+lw+11] = (0,0,0)
img_3[yo-lw-10:yo+lw+11,xo-lw+1:xo+lw] = (0,0,0)


for i in range(0,14):
	cv2.line(img_3,(xo-lw+4*i+3,yo+lw),(xo-lw+4*i+3,yo+lw+10),yellow,th)

for i in range(0,14):
	cv2.line(img_3,(xo-lw+4*i+3,yo-lw),(xo-lw+4*i+3,yo-lw-10),yellow,th)

for i in range(0,14):
	cv2.line(img_3,(xo-lw,yo-lw+4*i+3),(xo-lw-10,yo-lw+4*i+3),yellow,th)
for i in range(0,14):
	cv2.line(img_3,(xo+lw,yo-lw+4*i+3),(xo+lw+10,yo-lw+4*i+3),yellow,th)

cv2.imshow('3 Channel Window', img_3)
print("image shape: ", img_3.shape)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("circuit.png", img_3)
cv2.imwrite("circuit.bmp", img_3)
cv2.imwrite("map.png", img_3)





