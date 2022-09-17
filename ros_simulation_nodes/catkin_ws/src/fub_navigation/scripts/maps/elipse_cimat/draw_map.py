import cv2
import numpy as np
h = 222
w = 276 
lw = 30
a = 15
b = 60
c = 90
th = 1

H = 400 
W = 700
xo = int(W-w)/2
yo = int(H-h)/2


def draw_contour(img,ll,wd,col,sc=1): 
    cv2.line(img,(sc*(wd+xo),sc*(ll+yo)),(sc*(w-wd-1+xo),sc*(ll+yo)),col,th)
    cv2.line(img,(sc*(w-wd-1+xo),sc*(ll+yo)),(sc*(w-1-ll+xo),sc*(wd+yo)),col,th)
    cv2.line(img,(sc*(w-1-ll+xo),sc*(wd+yo)),(sc*(w-1-ll+xo),sc*(h-wd+yo)),col,th)
    cv2.line(img,(sc*(w-1-ll+xo),sc*(h-wd+yo)),(sc*(w-wd+xo),sc*(h-ll-1+yo)),col,th)
    cv2.line(img,(sc*(w-wd+xo),sc*(h-ll-1+yo)),(sc*(wd+xo),sc*(h-ll-1+yo)),col,th)
    cv2.line(img,(sc*(wd+xo),sc*(h-ll-1+yo)),(sc*(ll+xo),sc*(h-wd-1+yo)),col,th)
    cv2.line(img,(sc*(ll+xo),sc*(h-wd-1+yo)),(sc*(ll+xo),sc*(wd+yo)),col,th)
    cv2.line(img,(sc*(ll+xo),sc*(wd+yo)),(sc*(wd+xo),sc*(ll+yo)),col,th)
    return img 
rf = 1
yellow = (0,255,255)

img_3 = np.zeros([rf*H,rf*W,3],dtype=np.uint8)
img_3.fill(0)
img_3 = draw_contour(img_3,0*lw,a,yellow,rf)
img_3 = draw_contour(img_3,1*lw,b,yellow,rf)
img_3 = draw_contour(img_3,2*lw,c,yellow,rf)



# or img[:] = 255
cv2.imshow('3 Channel Window', img_3)
print("image shape: ", img_3.shape)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("map.png", img_3)


rf = 4
th = 2
img_3 = np.zeros([rf*H,rf*W,3],dtype=np.uint8)
img_3.fill(0)
img_3 = draw_contour(img_3,0*lw,a,yellow,rf)
img_3 = draw_contour(img_3,1*lw,b,yellow,rf)
img_3 = draw_contour(img_3,2*lw,c,yellow,rf)

cv2.imshow('3 Channel Window', img_3)
print("image shape: ", img_3.shape)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("map_sf.png", img_3)
cv2.imwrite("map_sf.bmp", img_3)