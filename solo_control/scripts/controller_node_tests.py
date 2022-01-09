#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped,PoseStamped
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose2D, Twist
from tf2_ros import buffer
from std_msgs.msg import Float64
import math


#Implementar a parte de inscricao no topico de goal positions

#Implementar a funcao de calculo da IK baseado nas goal positions/tamanho do step
def IK(x,y):
    #global th1
    #global th2
    l1 = 0.16
    l2 = 0.16
    alf = math.atan(-y/x)
    th2 = -math.acos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    gama = math.acos((x**2+y**2+l1**2-l2**2)/(2*l1*math.sqrt(x**2+y**2)))
    th1 = alf + gama
    
    return th1, th2



#StateStimation
#Pegar o estado atual de cada junta

#Trajetoria de Passo da Perna X
#Dado um step, calcular as novas pos X Y dado o X Y atual
#def passo(s=0.1,perna):
    #th1_atual (state estimation)
    #th2_atual (state estimation)
    #DK > x,y_atual
    #mantendo x cte e fazendo y = y_atual + s
    # th1, th2 = IK(x_new,y_new)
    


#Movimento da base e adaptacao das pernas
#Movimenta a base
#Adapta a posicao de cada perna (utilizando a pos atual) para "tras" com o IK
    
if __name__ == '__main__':

    #inicia o node
    rospy.init_node('controller_node')
    rate = rospy.Rate(20000) # 10hz

    pub1 = rospy.Publisher('/solo8/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/solo8/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/solo8/joint3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/solo8/joint4_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/solo8/joint5_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/solo8/joint6_position_controller/command', Float64, queue_size=10)
    pub7 = rospy.Publisher('/solo8/joint7_position_controller/command', Float64, queue_size=10)
    pub8 = rospy.Publisher('/solo8/joint8_position_controller/command', Float64, queue_size=10)

    pub_base = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    #Cria Posicao Inicial
    cmd1 = Float64()
    cmd1.data = 0.5
    cmd2 = Float64()
    cmd2.data = -1.0

    #Posicao apos o passo
    pas1 = Float64()
    pas1.data = 0.0302
    pas2 = Float64()
    pas2.data = -0.7447

    #Perna no ar
    ar1 = Float64()
    ar1.data = -0.10
    ar2 = Float64()
    ar2.data = -0.85

    #Movimento da Base
    bas1 = ModelState()
    bas1.model_name = 'solo8'
    bas1.reference_frame = 'base_link'
    bas1.pose.position.x = 0.05

    #Recuo
    rec1 = Float64()
    rec1.data = 0.5
    rec2 = Float64()
    rec2.data = -1.0   

    while not rospy.is_shutdown():

        th1,th2 = IK(0.2808,-0.1)

        print(th1,th2)

        rospy.sleep(0.5)

        pub1.publish(cmd1)
        pub2.publish(cmd2)
        pub3.publish(cmd1)
        pub4.publish(cmd2)
        pub5.publish(cmd1)
        pub6.publish(cmd2)
        pub7.publish(cmd1)
        pub8.publish(cmd2)

        rospy.sleep(0.5)

        #pub1.publish(ar1)
        #pub2.publish(ar2)

        #rospy.sleep(1)

        pub1.publish(pas1)
        pub2.publish(pas2)

        rospy.sleep(0.5)

        #pub_base.publish(bas1)
        #pub1.publish(rec1)
        #pub2.publish(rec2)

        #rospy.sleep(1)

        #pub7.publish(ar1)
        #pub8.publish(ar2)

        rospy.sleep(0.5)

        pub7.publish(pas1)
        pub8.publish(pas2)

        rospy.sleep(0.5)

        #pub3.publish(ar1)
        #pub4.publish(ar2)

        #rospy.sleep(1)

        pub3.publish(pas1)
        pub4.publish(pas2)

        rospy.sleep(0.5)

        #pub5.publish(ar1)
        #pub6.publish(ar2)

        #rospy.sleep(1)

        pub5.publish(pas1)
        pub6.publish(pas2)

        rospy.sleep(0.5)

        pub_base.publish(bas1)
        pub1.publish(rec1)
        pub2.publish(rec2)
        pub3.publish(rec1)
        pub4.publish(rec2)
        pub5.publish(rec1)
        pub6.publish(rec2)
        pub7.publish(rec1)
        pub8.publish(rec2)

        rospy.sleep(0.5)

        pub1.publish(cmd1)
        pub3.publish(cmd1)
        pub5.publish(cmd1)
        pub7.publish(cmd1)

        #rospy.sleep(1)
            
        pub2.publish(cmd2)
        pub4.publish(cmd2)
        pub6.publish(cmd2)
        pub8.publish(cmd2)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()



