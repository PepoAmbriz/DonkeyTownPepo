import rospy
import rospkg
import cv2
import math
import numpy as np
from tf.transformations import euler_from_quaternion

class MapConfig(object):
    def __init__(self,map_name,look_ahead='50cm'): 
        rospack = rospkg.RosPack()
        path = rospack.get_path('fub_navigation')+'/scripts/maps/'+map_name+'/'
        map_img = cv2.imread(path+'map.png')
        (h,w,l) = map_img.shape
        self.map_size_x = w  # cm
        self.map_size_y = h  # cm
        self.resolution = 1  #More than an argument, it's a parameter to meet. 

       #Do not charge all of     them 

        self.matrix_lane_1 = np.load(path + 'matrix'+look_ahead+'_lane2.npy')
        self.matrix_lane_2 = np.load(path + 'matrix'+look_ahead+'_lane1.npy')
        #self.matrix_lane_3 = np.load(path + 'matrix'+look_ahead+'_lane3.npy')
        self.matrix_nlane_1 = np.load(path + 'matrix-'+look_ahead+'_lane2.npy')
        self.matrix_nlane_2 = np.load(path + 'matrix-'+look_ahead+'_lane1.npy')
        #self.matrix_nlane_3 = np.load(path + 'matrix-'+look_ahead+'_lane3.npy')
        self.distance_lane_1 = np.load(path + 'matrix0cm_lane2.npy')
        self.distance_lane_2 = np.load(path + 'matrix0cm_lane1.npy')
        #self.distance_lane_3 = np.load(path + 'matrix0cm_lane3.npy')

class VectorfieldController(MapConfig):
    def __init__(self,map_name,lane,look_ahead):
        print(map_name,lane,look_ahead)
        super(VectorfieldController,self).__init__(map_name,look_ahead)
        
        self.lane = lane
        
        self.x = 0.0
        self.y = 0.0 

        self.last_lane_change = rospy.Time.now()
        
        self.Ks = [5.0,0.0,1.0]
        self.last_var = [0.0,0.0]
        self.last_time = rospy.Time.now()
        self.integral_error = 0.0

        if self.lane == 1:
            self.matrix = self.matrix_lane_1
            self.rmatrix = self.matrix_nlane_1
        else:
            self.matrix = self.matrix_nlane_2
            self.rmatrix = self.matrix_lane_2

    def get_coords_from_vf(self,raw_x, raw_y, matrix):
        #To match vector field
        x = raw_x+self.map_size_x/200.0
        y = raw_y+self.map_size_y/200.0
        x_index_floor = int(math.floor(x * (100.0 / self.resolution))) #Converting to cm
        y_index_floor = int(math.floor(y * (100.0 / self.resolution)))

        x_index_ceil = x_index_floor + 1
        y_index_ceil = y_index_floor + 1

        ceil_ratio_x = x * (100.0 / self.resolution) - x_index_floor #To check how much interpolate between floor and ceil.
        ceil_ratio_y = y * (100.0 / self.resolution) - y_index_floor

        if x_index_floor < 0:
            x_index_floor = 0
        if x_index_floor > self.map_size_x / self.resolution - 1:
            x_index_floor = self.map_size_x / self.resolution - 1

        if y_index_floor < 0:
            y_index_floor = 0
        if y_index_floor > self.map_size_y / self.resolution - 1:
            y_index_floor = self.map_size_y / self.resolution - 1

        if x_index_ceil < 0:
            x_index_ceil = 0
        if x_index_ceil > self.map_size_x / self.resolution - 1:
            x_index_ceil = self.map_size_x / self.resolution - 1

        if y_index_ceil < 0:
            y_index_ceil = 0
        if y_index_ceil > self.map_size_y / self.resolution - 1:
            y_index_ceil = self.map_size_y / self.resolution - 1

        vx_floor, vy_floor = matrix[x_index_floor, y_index_floor, :]
        vx_ceil, vy_ceil = matrix[x_index_ceil, y_index_ceil, :]

        vx = vx_floor * (1.0 - ceil_ratio_x) + vx_ceil * ceil_ratio_x
        vy = vy_floor * (1.0 - ceil_ratio_y) + vy_ceil * ceil_ratio_y
        return (vx,vy)

    def ddr_control(self,pose_msg,speed_value=None):
        dt = (pose_msg.header.stamp - self.last_time).to_sec()
        # 25hz
        if dt < 0.04:
            return (None, None,None)
        self.last_time = pose_msg.header.stamp
        self.x = pose_msg.pose.pose.position.x #Actual coord-x
        self.y = pose_msg.pose.pose.position.y #Actual coord-y
        orientation_q = pose_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list) #Actual yaw
        delta_xd,delta_yd = self.get_coords_from_vf(self.x,self.y,self.matrix)
        #eccentric point ddr controller
        ps = delta_xd+self.x
        qs = delta_yd+self.y
        dps = (ps-self.last_var[0])/dt
        dqs = (qs-self.last_var[1])/dt
        self.last_var[0] = ps
        self.last_var[1] = qs
        nu1 = dps+self.Ks[0]*(delta_xd)
        nu2 = dqs+self.Ks[1]*(delta_yd)
        speed = np.cos(yaw)*nu1+np.sin(yaw)*nu2
        steering = 14.2857*(-np.sin(yaw)*nu1+np.cos(yaw)*nu2) #where 14.2857 = 1/mu, mu:eccentrecity. Taking mu=0.07
        return(speed,steering,None)

    def steering_control(self, pose_msg, speed_value):
        dt = (pose_msg.header.stamp - self.last_time).to_sec()
        # 25hz
        if dt < 0.04:
            return (None,None,None)
        self.last_time = pose_msg.header.stamp
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        orientation_q = pose_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x3,y3 = self.get_coords_from_vf(self.x,self.y,self.matrix)
        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3

        angle = np.arctan2(f_y, f_x)

        self.integral_error = self.integral_error + angle * dt
        steering = self.Ks[0] * angle + self.Ks[2] * ((angle - self.last_var[0]) / dt) + self.Ks[1] * self.integral_error
        self.last_var[0] = angle

        if f_x > 0:
            speed = speed_value
        else:
            speed = speed_value

        gain = 1.0+1.0*np.exp(-10.0*abs(steering))
        if f_x > 0:
            speed = max(speed_value*gain, gain*(speed* ((np.pi / 3) / (abs(steering) + 1))))
        return (speed,steering,angle)

    def lane_change(self):
        if (rospy.Time.now() - self.last_lane_change).to_sec() < 1.0:
            return
        if self.lane == 1:
            self.matrix = self.matrix_lane_2
            self.lane = 2
        else:
            self.matrix = self.matrix_lane_1
            self.lane = 1
        self.last_lane_change = rospy.Time.now()
        print("lane change")