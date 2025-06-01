#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
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

    # Detener el robot
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)

def girar_angulo(pub, angulo_grados, velocidad_angular=math.radians(30)):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = velocidad_angular if angulo_grados > 0 else -velocidad_angular

    duracion = abs(math.radians(angulo_grados)) / abs(velocidad_angular)
    mover_robot(pub, 0, vel_msg.angular.z, duracion)

def main():
    rospy.init_node('movimiento_robot')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    time.sleep(1)

    if len(sys.argv) < 2:
        print("Uso: rosrun p3_pkg movimiento.py <modo>")
        return

    modo = int(sys.argv[1])

    if modo == 0:
        # Avanza 2 metros a 0.2 m/s
        mover_robot(pub, 0.2, 0, 10)

    elif modo == 1:
        # Triángulo equilátero de 3 m por lado
        for _ in range(3):
            mover_robot(pub, 0.2, 0, 15)  # 3 m a 0.2 m/s
            girar_angulo(pub, 120)

    elif modo == 2:
        # Cuadrado de 1 m por lado
        for i in range (10):
            for _ in range(4):
                mover_robot(pub, 0.2, 0, 5)  # 1 m
                girar_angulo(pub, 90)
            rospy.sleep(1)

    elif modo == 3:
        # Primer segmento
        mover_robot(pub, 0.2, 0, 2.5)
        
        # Segundo segmento
        girar_angulo(pub, 60)
        mover_robot(pub, 0.2, 0, 2.5)
        
        # Tercer segmento
        girar_angulo(pub, -120)
        mover_robot(pub, 0.2, 0, 2.5)
        
        # Cuarto segmento
        girar_angulo(pub, 60)
        mover_robot(pub, 0.2, 0, 2.5)
        
        # Quinto segmento
        girar_angulo(pub, 60)
        mover_robot(pub, 0.2, 0, 2.5)
        
        # Sexto segmento:
        girar_angulo(pub, -120)
        mover_robot(pub, 0.2, 0, 2.5)

    else:
        print("Modo inválido. Usa 0, 1, 2 o 3.")

if __name__ == '__main__':
    main()

