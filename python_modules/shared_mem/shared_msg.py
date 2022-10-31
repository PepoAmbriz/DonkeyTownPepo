import shared_mem as shmem
import os
import geometry_msgs.msg as geo_msg
from  donkietown_msgs.msg import CooperativeAwarenessMessage as CAM 
from std_msgs.msg import Header, Time 

class SharedPose2D:
      def __init__(self,name):
            self.x = shmem.MapedFloat(name+'/x')
            self.y = shmem.MapedFloat(name+'/y')
            self.theta = shmem.MapedFloat(name+'/theta')
            self.path = os.path.dirname(self.x.path)
      def set(self,lpose2D):
            self.x.set(lpose2D.x)
            self.y.set(lpose2D.y)
            self.theta.set(lpose2D.theta)
      def get(self):
            lpose2D = geo_msg.Pose2D()
            lpose2D.x = self.x.get()
            lpose2D.y = self.y.get()
            lpose2D.theta = self.theta.get()
            return lpose2D
      def delete(self):
            self.x.delete()
            self.y.delete()
            self.theta.delete()
            os.rmdir(self.path)

class SharedTime:
      def __init__(self,name):
            self.secs = shmem.MapedInt(name+'/secs')
            self.nsecs = shmem.MapedInt(name+'/nsecs')
            self.path = os.path.dirname(self.secs.path)
      def set(self,ltime):
            self.secs.set(ltime.secs)
            self.nsecs.set(ltime.nsecs)
      def get(self):
            ltime = Time()
            ltime.data.secs = self.secs.get()
            ltime.data.nsecs = self.nsecs.get()
            return ltime
      def delete(self):
            self.secs.delete()
            self.nsecs.delete()
            os.rmdir(self.path)

class SharedCAMHeader:
      def __init__(self,name):
            self.stamp = SharedTime(name+'/stamp')
            self.car_id = shmem.MapedShortInt(name+'/car_id')
            self.path = os.path.dirname(self.stamp.path)
      def set(self, lheader):
            self.stamp.set(lheader.stamp)
            self.car_id.set(int(lheader.frame_id))
      def get(self):
            lheader = Header()
            lheader.stamp = self.stamp.get()
            lheader.frame_id = str(self.car_id.get())
            return lheader
      def delete(self):
            self.stamp.delete()
            self.car_id.delete()
            os.rmdir(self.path)


class SharedCAM:
      def __init__(self,name):
            self.header = SharedCAMHeader(name+'/header')
            self.reference_pose = SharedPose2D(name+'/reference_pose')
            self.speed = shmem.MapedFloat(name+'/speed')
            self.lane = shmem.MapedShortInt(name+'/lane')
            self.drive_dir = shmem.MapedBool(name+'/drive_direction')
            self.heading = shmem.MapedFloat(name+'/heading')
            self.lane_change = shmem.MapedBool(name+'/lane_change')
            self.path = os.path.dirname(self.reference_pose.path)
      def set(self,lCAM):
            self.header.set(lCAM.header)
            self.reference_pose.set(lCAM.reference_pose)
            self.speed.set(lCAM.speed)
            self.lane.set(lCAM.lane)
            self.drive_dir.set(lCAM.drive_direction)
            self.heading.set(lCAM.heading)
            self.lane_change.set(lCAM.lane_change)
      def get(self):
            lCAM = CAM()
            lCAM.header = self.header.get()
            lCAM.reference_pose = self.reference_pose.get()
            lCAM.speed = self.speed.get()
            lCAM.lane = self.lane.get()
            lCAM.drive_direction = self.drive_dir.get()
            lCAM.heading = self.heading.get()
            lCAM.lane_change = self.lane_change.get()
            return lCAM
      def delete(self):
            self.header.delete()
            self.reference_pose.delete()
            self.speed.delete()
            self.lane.delete()
            self.drive_dir.delete()
            self.heading.delete()
            self.lane_change.delete()
            os.rmdir(self.path)
