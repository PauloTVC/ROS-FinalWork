#! /usr/bin/env python2.7

import rospy
from rospy.topics import Subscriber
import tf2_ros
from math import  sqrt
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose2D, Twist
from tf2_ros import buffer
from ros_final.msg import PointStampedArray


#Input Speed of Cente of Mass by 
#It will be feed by the Main 

def Spd(data):
     speed = Twist()
     speed = rospy.wait_for_message("speed", Twist)
     return speed

#Create the PointStampeds
def vect(xp, yp, zp):
     po = PointStamped()
     po.point.x = xp
     po.point.y = yp
     po.point.z = zp
     
     return po


def main():
     rospy.init_node('Point_Planning')
     buffer = tf2_ros.Buffer()
     listener = tf2_ros.TransformListener(buffer)
     rospy.Subscriber("speed", Twist, Spd)
     P = rospy.Publisher("points", PointStampedArray, queue_size=10)
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

     #Array of PoinStamped
     pv = PointStampedArray
     pv.header.frame_id="base_link"
     pv.header.stamp = pe 


     #hidden leg right
     pv.points[0] = vect(s -0.08*sqrt(2), w/2, h)
     #hidden leg left
     pv.points[1] = vect(s -0.08*sqrt(2), -w/2, h)
     #forward leg right
     pv.points[2] = vect(s +0.08*sqrt(2), w/2, h)
     #forward leg left
     pv.points[3] = vect(s +0.08*sqrt(2), -w/2, h)

     #publishing
     P.publish(pv)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
