#! /usr/bin/env python2.7

import rospy
from geometry_msgs.msg import  Twist



def main():
    pub = rospy.Publisher('speed', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    s = Twist()
    s.linear.x = 10
    s.linear.y = 0
    s.linear.z = 0
    s.angular.x = 0
    s.angular.y = 0
    s.angular.z = 0

    

    while not rospy.is_shutdown():
        print(s.linear.x)
        pub.publish(s)
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass