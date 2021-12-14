#! /usr/bin/env python2.7

import rospy
from rospy.topics import Subscriber
import tf2_ros
from math import  sqrt
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Twist
from tf2_ros import buffer
from ros_final.msg import PointStampedArray

spd = Twist()
P = rospy.Publisher("points", PointStampedArray, queue_size=10)


#Input Speed of Cente of Mass by 
#It will be feed by the Main 

#callback
def Spd(data):
     #spd = rospy.wait_for_message("speed", Twist)
     #print(data)
     spd = data
     #height
     h = -0.16*sqrt(2)
     #width
     w = 0.2996
     #step
     s = 0.05

     
     if spd.linear.x == 0:
          spd.linear.x = 0.001
     #period
     v = spd.linear.x
     pe = s/(4*v)

     #Array of PoinStamped
     vpt = PointStampedArray()
     vpt.header.frame_id="base_link"
     vpt.period = pe 

     pv = []

     #hidden leg right
     pv.append(ptd(-0.08*sqrt(2), w/2, h))
     pv.append(ptd(s/2 -0.08*sqrt(2), w/2, h + 0.05))     
     pv.append(ptd(s -0.08*sqrt(2), w/2, h))
     #hidden leg left
     pv.append(ptd(-0.08*sqrt(2), -w/2, h))
     pv.append(ptd(s/2 -0.08*sqrt(2), -w/2, h + 0.05))
     pv.append(ptd(s -0.08*sqrt(2), -w/2, h))
     #forward leg right
     pv.append(ptd(0.08*sqrt(2), w/2, h))
     pv.append(ptd(s/2 +0.08*sqrt(2), w/2, h + 0.05))     
     pv.append(ptd(s +0.08*sqrt(2), w/2, h))
     #forward leg left
     pv.append(ptd(0.08*sqrt(2), -w/2, h))
     pv.append(ptd(s/2 +0.08*sqrt(2), -w/2, h + 0.05))
     pv.append(ptd(s +0.08*sqrt(2), -w/2, h))

     vpt.points = pv

     print(pv)
     print("------------------------")

     #publishing
     P.publish(vpt)
     

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

     
     r = rospy.Rate(10)

     rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
