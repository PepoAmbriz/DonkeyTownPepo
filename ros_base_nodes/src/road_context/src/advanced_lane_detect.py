#!/usr/bin/env python2

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo 
from cv_bridge import CvBridge, CvBridgeError
import sys

import numpy as np

from calibration_utils import calibrate_camera, undistort
from binarization_utils import binarize
from perspective_utils import birdeye
from line_utils import get_fits_by_sliding_windows, draw_back_onto_the_road, Line, get_fits_by_previous_fits
from globals import xm_per_pix, time_window
 
def prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter):
    """
    Prepare the final pretty pretty output blend, given all intermediate pipeline images

    :param blend_on_road: color image of lane blend onto the road
    :param img_binary: thresholded binary image
    :param img_birdeye: bird's eye view of the thresholded binary image
    :param img_fit: bird's eye view with detected lane-lines highlighted
    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param offset_meter: offset from the center of the lane
    :return: pretty blend with all images and stuff stitched
    """
    h, w = blend_on_road.shape[:2]

    thumb_ratio = 0.2
    thumb_h, thumb_w = int(thumb_ratio * h), int(thumb_ratio * w)

    off_x, off_y = 20, 15

    # add a gray rectangle to highlight the upper area
    mask = blend_on_road.copy()
    mask = cv2.rectangle(mask, pt1=(0, 0), pt2=(w, thumb_h+2*off_y), color=(0, 0, 0), thickness=cv2.FILLED)
    blend_on_road = cv2.addWeighted(src1=mask, alpha=0.2, src2=blend_on_road, beta=0.8, gamma=0)

    # add thumbnail of binary image
    thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    thumb_binary = np.dstack([thumb_binary, thumb_binary, thumb_binary]) * 255
    blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary

    # add thumbnail of bird's eye view
    thumb_birdeye = cv2.resize(img_birdeye, dsize=(thumb_w, thumb_h))
    thumb_birdeye = np.dstack([thumb_birdeye, thumb_birdeye, thumb_birdeye]) * 255
    blend_on_road[off_y:thumb_h+off_y, 2*off_x+thumb_w:2*(off_x+thumb_w), :] = thumb_birdeye

    # add thumbnail of bird's eye view (lane-line highlighted)
    thumb_img_fit = cv2.resize(img_fit, dsize=(thumb_w, thumb_h))
    blend_on_road[off_y:thumb_h+off_y, 3*off_x+2*thumb_w:3*(off_x+thumb_w), :] = thumb_img_fit

    # add text (curvature and offset info) on the upper right of the blend
    mean_curvature_meter = np.mean([line_lt.curvature_meter, line_rt.curvature_meter])
    print(mean_curvature_meter)
    #font = cv2.FONT_HERSHEY_SIMPLEX
    #cv2.putText(blend_on_road, 'Curvature radius: {:.02f}m'.format(mean_curvature_meter), (860, 60), font, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
    #cv2.putText(blend_on_road, 'Offset from center: {:.02f}m'.format(offset_meter), (860, 130), font, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    return blend_on_road

def compute_offset_from_center(line_lt, line_rt, frame_width):
        """
        Compute offset from center of the inferred lane.
        The offset from the lane center can be computed under the hypothesis that the camera is fixed
        and mounted in the midpoint of the car roof. In this case, we can approximate the car's deviation
        from the lane center as the distance between the center of the image and the midpoint at the bottom
        of the image of the two lane-lines detected.

        :param line_lt: detected left lane-line
        :param line_rt: detected right lane-line
        :param frame_width: width of the undistorted frame
        :return: inferred offset
        """
        if line_lt.detected and line_rt.detected:
            line_lt_bottom = np.mean(line_lt.all_x[line_lt.all_y > 0.95 * line_lt.all_y.max()])
            line_rt_bottom = np.mean(line_rt.all_x[line_rt.all_y > 0.95 * line_rt.all_y.max()])
            lane_width = line_rt_bottom - line_lt_bottom
            midpoint = frame_width / 2
            offset_pix = abs((line_lt_bottom + lane_width / 2) - midpoint)
            offset_meter = xm_per_pix * offset_pix
        else:
            offset_meter = -1

        return offset_meter


class advanced_lanedet_node:
	def __init__(self,img_topic):
		self.processed_frames = 0
		self.line_lt = Line(buffer_len = time_window)
		self.line_rt = Line(buffer_len = time_window)
		self.keep_state = False

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(img_topic+"image_raw",Image,self.on_img)
		self.cmodel_sub = rospy.Subscriber(img_topic+"camera_info", CameraInfo,self.on_cam_info)
		self.cmodel_mat = [[]]
		self.cmodel_dist = []
		self.image_pub = rospy.Publisher("/AutoModelMini/lane/img",Image,queue_size=1)
		


	def on_img(self,img_msg):
		try: 
			cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		# undistort the image using coefficients found in calibration
		img_undistorted = undistort(cv_img, self.cmodel_mat,self.cmodel_dist,verbose=False)	

    	# binarize the frame s.t. lane lines are highlighted as much as possible
		img_binary = binarize(img_undistorted, verbose=False)

		# compute perspective transform to obtain bird's eye view
		#birdeye roi is hardcoded
		img_birdeye, M, Minv = birdeye(img_binary, verbose=False)
		# fit 2-degree polynomial curve onto lane lines found

		
		if self.processed_frames > 0 and self.keep_state and self.line_lt.detected and self.line_rt.detected:
			self.line_lt, self.line_rt, img_fit = get_fits_by_previous_fits(img_birdeye, self.line_lt, self.line_rt, verbose=False)
		else:
			self.line_lt, self.line_rt, img_fit = get_fits_by_sliding_windows(img_birdeye, self.line_lt, self.line_rt, n_windows=9, verbose=False)

		# compute offset in meter from center of the lane
		offset_meter = compute_offset_from_center(self.line_lt, self.line_rt, frame_width=cv_img.shape[1])
        #print(offset_meter)

        # draw the surface enclosed by lane lines back onto the original frame
        blend_on_road = draw_back_onto_the_road(img_undistorted, Minv, self.line_lt, self.line_rt, self.keep_state)

        # stitch on the top of final output images from different steps of the pipeline
        blend_output = prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, self.line_lt, self.line_rt, offset_meter)

        self.processed_frames += 1

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(blend_output,"bgr8"))
        except CvBridgeError as e: 
            print(e)

	def on_cam_info(self,cam_info_msg): 
		self.cmodel_mat = np.reshape(cam_info_msg.K,(3,3))
		self.cmodel_dist = np.asarray(cam_info_msg.D)
		self.cmodel_sub.unregister()


		
def main(args): 
	rospy.init_node('AdvancedLaneDetector', anonymous=True)
	node = advanced_lanedet_node("/app/camera/rgb/")
	try:
		rospy.spin()
	except KeyboardInterrupt: 
		print("Shutting down")
	

if __name__ == '__main__': 
	main(sys.argv)