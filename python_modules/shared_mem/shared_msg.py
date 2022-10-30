import shared_mem as shmem
import os
import geometry_msgs.msg as geo_msg

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


class SharedCAM:
      def __init__(self,name):
            self.ref_pos = SharedPose2D(name+'/reference_position')
            self.speed = shmem.MapedFloat(name+'/speed')
            self.lane = shmem.MapedShortInt(name+'/lane')
            self.drive_dir = shmem.MapedBool(name+'/drive_direction')
            self.heading = shmem.MapedFloat(name+'/heading')
            self.lane_change = shmem.MapedBool(name+'/lane_change')
            self.path = os.path.dirname(self.ref_pos.path)
      def delete(self):
            self.ref_pos.delete()
            self.speed.delete()
            self.lane.delete()
            self.drive_dir.delete()
            self.heading.delete()
            self.lane_change.delete()
            os.rmdir(self.path)