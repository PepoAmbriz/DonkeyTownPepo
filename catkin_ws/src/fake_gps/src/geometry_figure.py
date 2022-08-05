import numpy as np

def order_corners(pts):
	xSorted = pts[np.argsort(pts[:,0]),:]
	lMost = xSorted[:2,:]
	rMost = xSorted[2:,:]
	(tl,bl) = lMost[np.argsort(lMost[:,1]),:]
	(tr,br) = rMost[np.argsort(rMost[:,1]),:]
	return np.array([tl,tr,br,bl], dtype="int32")

def comp_apo(pts): 
	center = np.mean(pts,axis=1)
	apos = [np.linalg.norm(center-p) for p in pts]
	return apos/np.mean(apos)

def comp_sides(pts):
	pts = np.reshape(pts,(4,2))
	p0 = pts[-1]
	sides = []
	for p in pts: 
		sides.append(np.linalg.norm(p-p0))
		p0 = p
	return sides

def comp_diags(pts):
	pts = np.reshape(pts,(4,2))
	diags = []
	for i in range(0,len(pts),2): 
		diags.append(np.linalg.norm(pts[i]-pts[i+1])) #Esta bien?
	return diags

def square_var(pts): 
	sides = comp_sides(pts)
	diags = comp_diags(pts)
	sides_var = np.var(sides/np.mean(sides))
	diags_var = np.var(diags/np.mean(diags))
	return np.array([sides_var,diags_var])