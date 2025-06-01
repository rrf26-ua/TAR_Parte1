#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
import time

def mover_robot(publisher, vel_lineal, vel_angular, tiempo):
    vel_msg = Twist()
    vel_msg.linear.x = vel_lineal
    vel_msg.angular.z = vel_angular
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)
    while (rospy.Time.now().to_sec() - t0) < tiempo:
        publisher.publish(vel_msg)
        rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)

def girar_angulo(pub, angulo_grados, velocidad_angular=math.radians(30)):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    
    if angulo_grados >= 0:
        vel_msg.angular.z = abs(velocidad_angular)
    else:
        vel_msg.angular.z = -abs(velocidad_angular)
    duracion = abs(math.radians(angulo_grados)) / abs(velocidad_angular)
    mover_robot(pub, 0, vel_msg.angular.z, duracion)

def main():
    rospy.init_node('aparcamiento')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    time.sleep(1)

    rospy.loginfo("Iniciando maniobra de aparcamiento con Turtlebot 3")

    mover_robot(pub, 0.2, 0, 7.5)
    girar_angulo(pub, 90)
    mover_robot(pub, -0.2, 0, 7.1)
    
    rospy.loginfo("Maniobra de aparcamiento completada.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

