import cv2
import numpy as np


h = 179
w = 239 

H = 400 
W = 700

sc = 4
def draw_arc(image,r,cx,cy,color,angs,th=1):
    height, width = image.shape[0:2]
    # Ellipse parameters
    radius = r
    center = (width/2+cx, height/2-cy)
    print(center)
    axes = (radius, radius)
    angle = 0
    startAngle = angs[0]
    endAngle = angs[1]
    # When thickness == -1 -> Fill shape
    

    # Draw black half circle
    cv2.ellipse(image, center, axes, angle, startAngle, endAngle, color, th)

    #axes = (radius-2, radius-2)
    # Draw a bit smaller white half circle
    #cv2.ellipse(image, center, axes, angle, startAngle, endAngle, (0,0,0), thickness)

def centered_line(image,startp,endp,color,th=1): 
	height, width = image.shape[0:2]
	spcx = startp[0]
	spcy = startp[1]
	epcx = endp[0]
	epcy = endp[1]
	cstart = (width/2+spcx, height/2-spcy)
	cend = (width/2+epcx, height/2-epcy)
	return cv2.line(image,cstart,cend,color,th)

img_3 = np.zeros([H*sc,W*sc,3],dtype=np.uint8)
img_3.fill(0)
yellow = (0,255,255)

th = 4
draw_arc(img_3,90*sc,-50*sc,0,yellow,(90,270),th)
draw_arc(img_3,90*sc,50*sc,0,yellow,(270,360),th)
draw_arc(img_3,90*sc,50*sc,0,yellow,(0,90),th)

draw_arc(img_3,60*sc,-50*sc,0,yellow,(90,270),th)
draw_arc(img_3,60*sc,50*sc,0,yellow,(270,360),th)
draw_arc(img_3,60*sc,50*sc,0,yellow,(0,90),th)

draw_arc(img_3,120*sc,-50*sc,0,yellow,(90,270),th)
draw_arc(img_3,120*sc,50*sc,0,yellow,(270,360),th)
draw_arc(img_3,120*sc,50*sc,0,yellow,(0,90),th)

img_3 = centered_line(img_3,(-50*sc,90*sc),(50*sc,90*sc),yellow,th)
img_3 = centered_line(img_3,(-50*sc,60*sc),(50*sc,60*sc),yellow,th)
img_3 = centered_line(img_3,(-50*sc,120*sc),(50*sc,120*sc),yellow,th)

img_3 = centered_line(img_3,(-50*sc,-120*sc),(50*sc,-120*sc),yellow,th)
img_3 = centered_line(img_3,(-50*sc,-60*sc),(50*sc,-60*sc),yellow,th)
img_3 = centered_line(img_3,(-50*sc,-90*sc),(50*sc,-90*sc),yellow,th)

#img_3 = draw_contour(img_3,0*24,a,yellow)
#img_3 = draw_contour(img_3,1*24,b,yellow)
#img_3 = draw_contour(img_3,2*24,c,yellow)



# or img[:] = 255
cv2.imshow('3 Channel Window', img_3)
print("image shape: ", img_3.shape)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("circuit_sf.bmp", img_3)