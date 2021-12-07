import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import  Twist
from tf2_ros import buffer


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
        pub.publish(s)
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass