#!/usr/bin/env python

from os import tcgetpgrp
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped,PoseStamped
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import JointState
from tf2_ros import buffer
from std_msgs.msg import Float64
import math

cmd1 = Float64()
cmd2 = Float64()
bas1 = ModelState()

#Implementar a parte de inscricao no topico de goal positions
def goalstepCallback(msg):
    global step
    step = msg
    print(step)

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

def DK(th1,th2):
    l1 = 0.16
    l2= 0.16
    x = l1*math.cos(th1) + l2*math.cos(th1 + th2)
    y = -(l1*math.sin(th1) + l2*math.sin(th1 + th2))

    return x, y

#StateStimation
#Pegar o estado atual de cada junta
def stateCallback(msg):
    global pos_states
    pos_states = msg.position

#Trajetoria de Passo da Perna X
#Dado um step, calcular as novas pos X Y dado o X Y atual
def passo(perna, s=0.1,): #modeificar para funcionar para todas as pernas

    T = 0.05
    if perna==1:
        th1_current = pos_states[0]
        th2_current = pos_states[1]
        #DK > x,y_atual
        x_current, y_current = DK(th1_current, th2_current)
    
        #mantendo x cte e fazendo y = y_atual + s

        x_new = x_current #msm altura
        y_new = y_current + s

        #Sequencia intermediaria do swing foot
        x_int1 = x_current - 0.02
        y_int1 = s/4
        th1_int1, th2_int1 = IK(x_int1,y_int1)
        cmd1.data = th1_int1
        cmd2.data = th2_int1
        pub1.publish(cmd1)
        pub2.publish(cmd2)
        rospy.sleep(T)

        x_int2 = x_current - 0.03
        y_int2 = s/2
        th1_int2, th2_int2 = IK(x_int2,y_int2)
        cmd1.data = th1_int2
        cmd2.data = th2_int2
        pub1.publish(cmd1)
        pub2.publish(cmd2)
        rospy.sleep(T)

        x_int3 = x_current - 0.01
        y_int3 = 3*s/4
        th1_int3, th2_int3 = IK(x_int3,y_int3)
        cmd1.data = th1_int3
        cmd2.data = th2_int3
        pub1.publish(cmd1)
        pub2.publish(cmd2)
        rospy.sleep(T)

        th1_new, th2_new = IK(x_new,y_new)
        cmd1.data = th1_new
        cmd2.data = th2_new
        pub1.publish(cmd1)
        pub2.publish(cmd2)

    elif perna==2:
        th1_current = pos_states[2]
        th2_current = pos_states[3]
        #DK > x,y_atual
        x_current, y_current = DK(th1_current, th2_current)
    
        #mantendo x cte e fazendo y = y_atual + s

        x_new = x_current #msm altura
        y_new = y_current + s

        #Sequencia intermediaria do swing foot
        x_int1 = x_current - 0.02
        y_int1 = s/4
        th1_int1, th2_int1 = IK(x_int1,y_int1)
        cmd1.data = th1_int1
        cmd2.data = th2_int1
        pub3.publish(cmd1)
        pub4.publish(cmd2)
        rospy.sleep(T)

        x_int2 = x_current - 0.03
        y_int2 = s/2
        th1_int2, th2_int2 = IK(x_int2,y_int2)
        cmd1.data = th1_int2
        cmd2.data = th2_int2
        pub3.publish(cmd1)
        pub4.publish(cmd2)
        rospy.sleep(T)

        x_int3 = x_current - 0.01
        y_int3 = 3*s/4
        th1_int3, th2_int3 = IK(x_int3,y_int3)
        cmd1.data = th1_int3
        cmd2.data = th2_int3
        pub3.publish(cmd1)
        pub4.publish(cmd2)
        rospy.sleep(T)

        th1_new, th2_new = IK(x_new,y_new)
        cmd1.data = th1_new
        cmd2.data = th2_new
        #adicionar cases para as pernas
        pub3.publish(cmd1)
        pub4.publish(cmd2)   

    elif perna==3:
        th1_current = pos_states[4]
        th2_current = pos_states[5]
        #DK > x,y_atual
        x_current, y_current = DK(th1_current, th2_current)
    
        #mantendo x cte e fazendo y = y_atual + s

        x_new = x_current #msm altura
        y_new = y_current + s

        #Sequencia intermediaria do swing foot
        x_int1 = x_current - 0.02
        y_int1 = s/4
        th1_int1, th2_int1 = IK(x_int1,y_int1)
        cmd1.data = th1_int1
        cmd2.data = th2_int1
        pub5.publish(cmd1)
        pub6.publish(cmd2)
        rospy.sleep(T)

        x_int2 = x_current - 0.03
        y_int2 = s/2
        th1_int2, th2_int2 = IK(x_int2,y_int2)
        cmd1.data = th1_int2
        cmd2.data = th2_int2
        pub5.publish(cmd1)
        pub6.publish(cmd2)
        rospy.sleep(T)

        x_int3 = x_current - 0.01
        y_int3 = 3*s/4
        th1_int3, th2_int3 = IK(x_int3,y_int3)
        cmd1.data = th1_int3
        cmd2.data = th2_int3
        pub5.publish(cmd1)
        pub6.publish(cmd2)
        rospy.sleep(T)

        th1_new, th2_new = IK(x_new,y_new)
        cmd1.data = th1_new
        cmd2.data = th2_new
        #adicionar cases para as pernas
        pub5.publish(cmd1)
        pub6.publish(cmd2) 

    elif perna==4:
        th1_current = pos_states[6]
        th2_current = pos_states[7]
        #DK > x,y_atual
        x_current, y_current = DK(th1_current, th2_current)
    
        #mantendo x cte e fazendo y = y_atual + s

        x_new = x_current #msm altura
        y_new = y_current + s

        #Sequencia intermediaria do swing foot
        x_int1 = x_current - 0.02
        y_int1 = s/4
        th1_int1, th2_int1 = IK(x_int1,y_int1)
        cmd1.data = th1_int1
        cmd2.data = th2_int1
        pub7.publish(cmd1)
        pub8.publish(cmd2)
        rospy.sleep(T)

        x_int2 = x_current - 0.03
        y_int2 = s/2
        th1_int2, th2_int2 = IK(x_int2,y_int2)
        cmd1.data = th1_int2
        cmd2.data = th2_int2
        pub7.publish(cmd1)
        pub8.publish(cmd2)
        rospy.sleep(T)

        x_int3 = x_current - 0.01
        y_int3 = 3*s/4
        th1_int3, th2_int3 = IK(x_int3,y_int3)
        cmd1.data = th1_int3
        cmd2.data = th2_int3
        pub7.publish(cmd1)
        pub8.publish(cmd2)
        rospy.sleep(T)

        th1_new, th2_new = IK(x_new,y_new)
        cmd1.data = th1_new
        cmd2.data = th2_new
        #adicionar cases para as pernas
        pub7.publish(cmd1)
        pub8.publish(cmd2) 
    else:
        print('Insira um numero de 1 a 4')

#Movimento da base e adaptacao das pernas
#Movimenta a base
#Adapta a posicao de cada perna (utilizando a pos atual) para "tras" com o IK
def move_base(K=0.1):

    
    #print(k)
    bas1.model_name = 'solo8'
    bas1.reference_frame = 'base_link'

    k = K/4
    bas1.pose.position.x = k

    th1_current = pos_states[0]
    th2_current = pos_states[1]
    th3_current = pos_states[2]
    th4_current = pos_states[3]
    th5_current = pos_states[4]
    th6_current = pos_states[5]
    th7_current = pos_states[6]
    th8_current = pos_states[7]

    x1,y1 = DK(th1_current,th2_current)
    x2,y2 = DK(th3_current,th4_current)
    x3,y3 = DK(th5_current,th6_current)
    x4,y4 = DK(th7_current,th8_current)

    x1_new = x1 #msm altura
    y1_new = y1 - k
    x2_new = x2 #msm altura
    y2_new = y2 - k
    x3_new = x3 #msm altura
    y3_new = y3 - k
    x4_new = x4 #msm altura
    y4_new = y4 - k

    th1_new, th2_new = IK(x1_new,y1_new)
    th3_new, th4_new = IK(x2_new,y2_new)
    th5_new, th6_new = IK(x3_new,y3_new)
    th7_new, th8_new = IK(x4_new,y4_new)

    pub_base.publish(bas1)

    cmd1.data = th1_new
    cmd2.data = th2_new
    pub1.publish(cmd1)
    pub2.publish(cmd2)

    cmd1.data = th3_new
    cmd2.data = th4_new
    pub3.publish(cmd1)
    pub4.publish(cmd2)

    cmd1.data = th5_new
    cmd2.data = th6_new
    pub5.publish(cmd1)
    pub6.publish(cmd2)

    cmd1.data = th7_new
    cmd2.data = th8_new
    pub7.publish(cmd1)
    pub8.publish(cmd2)

    
if __name__ == '__main__':

    #inicia o node
    rospy.init_node('controller_node')
    rate = rospy.Rate(2000) # 10hz

    pub1 = rospy.Publisher('/solo8/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/solo8/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/solo8/joint3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/solo8/joint4_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/solo8/joint5_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/solo8/joint6_position_controller/command', Float64, queue_size=10)
    pub7 = rospy.Publisher('/solo8/joint7_position_controller/command', Float64, queue_size=10)
    pub8 = rospy.Publisher('/solo8/joint8_position_controller/command', Float64, queue_size=10)

    pub_base = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    state_subscriber = rospy.Subscriber('/solo8/joint_states', JointState, stateCallback)
    goalstep_subscriber = rospy.Subscriber('/solo8/goal_step', Float64, goalstepCallback)

    #Cria Posicao Inicial
    cmd1.data = 0.5
    cmd2.data = -1.0

    t = 0.1
    ci = 0

    while not rospy.is_shutdown():


        if ci == 0:
            rospy.sleep(0.4)
            pub1.publish(cmd1)
            pub2.publish(cmd2)
            pub3.publish(cmd1)
            pub4.publish(cmd2)
            pub5.publish(cmd1)
            pub6.publish(cmd2)
            pub7.publish(cmd1)
            pub8.publish(cmd2)
            rospy.sleep(0.4)
            ci=1

        for i in range(1,5):
            move_base(0.1)
            rospy.sleep(t)
            passo(i,0.1)
            rospy.sleep(t)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()



