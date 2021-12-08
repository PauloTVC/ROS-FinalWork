import rospy
from rospy.topics import Subscriber
import tf2_ros
from math import  atan2, hypot, sqrt
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose2D, Twist
from tf2_ros import buffer
from ros_final.msg import poseStp

#Input Speed of Cente of Mass by 
#It will be feed by the Main Node
def Spd(data):
     speed = Twist()
     speed = rospy.wait_for_message("speed", Twist)
     print()
     return speed


def vect(xp, yp, zp, time):
     po = PointStamped()
     po.header.frame_id = "base_link"
     po.header.stamp = time
     po.point.x = xp
     po.point.y = yp
     po.point.z = zp
     
     return po


def main():
     rospy.init_node('Point_Planning')
     buffer = tf2_ros.Buffer()
     listener = tf2_ros.TransformListener(buffer)
     rospy.Subscriber("speed", Twist, Spd)
     #P = rospy.Publisher("points",PointStamped, queue_size=10)
     r = rospy.Rate(10)

     #speed
     spd = Spd() 
     #height
     h = -0.16*sqrt(2)
     #width
     w = 0.2996
     #step
     s = 0.05 
     #period
     pe = s/(4*spd)

     pv = poseStp 


     pv[0] = vect(s -0.08*sqrt(2), w/2, h, pe)
     pv[1] = vect(s -0.08*sqrt(2), -w/2, h, pe)
     pv[2] = vect(s +0.08*sqrt(2), w/2, h, pe)
     pv[3] = vect(s +0.08*sqrt(2), -w/2, h, pe)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

