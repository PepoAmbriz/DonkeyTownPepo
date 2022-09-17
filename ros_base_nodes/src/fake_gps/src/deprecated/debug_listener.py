#dummy node for concept testing
import roslib 
import rospy 
from donkietown_msgs.msg import MarkerEdge, MarkerEdgeArray 

def callback(data): 
	print(data.header.stamp)

def listener():
	rospy.Subscriber('/fake_gps/marks_corners', MarkerEdgeArray, callback)
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('pylistener', anonymous=True)
	listener()