#! /usr/bin/env python2.7

import rospy
from rospy.topics import Subscriber
import tf2_ros
from math import  sqrt
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose2D, Twist
from tf2_ros import buffer
from ros_final.msg import PointStampedArray


global spd

spd = 0.001

#Input Speed of Cente of Mass by 
#It will be feed by the Main 


def Spd(data):
     
     spd = Twist()
     spd = rospy.wait_for_message("speed", Twist)
     

#Create the PointStampeds
def ptd(xp, yp, zp):
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

     #height
     h = -0.16*sqrt(2)
     #width
     w = 0.2996
     #step
     s = 0.05

     if spd == 0:
          spd = 0.001
     #period
     pe = s/(4*spd)

     #Array of PoinStamped
     vpt = PointStampedArray()
     vpt.header.frame_id="base_link"
     vpt.period = pe 

     pv = [12]


     #hidden leg right
     pv[0] = ptd(-0.08*sqrt(2), w/2, h)
     pv[1] = ptd(s/2 -0.08*sqrt(2), w/2, h + 0.05)     
     pv[2] = ptd(s -0.08*sqrt(2), w/2, h)
     #hidden leg left
     pv[3] = ptd(-0.08*sqrt(2), -w/2, h)
     pv[4] = ptd(s/2 -0.08*sqrt(2), -w/2, h + 0.05)
     pv[5] = ptd(s -0.08*sqrt(2), -w/2, h)
     #forward leg right
     pv[6] = ptd(0.08*sqrt(2), w/2, h)
     pv[7] = ptd(s/2 +0.08*sqrt(2), w/2, h + 0.05)     
     pv[8] = ptd(s +0.08*sqrt(2), w/2, h)
     #forward leg left
     pv[9] = ptd(0.08*sqrt(2), -w/2, h)
     pv[10] = ptd(s/2 +0.08*sqrt(2), -w/2, h + 0.05)
     pv[11] = ptd(s +0.08*sqrt(2), -w/2, h)

     vpt.points = pv



     #publishing
     P.publish(vpt)

     r.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
