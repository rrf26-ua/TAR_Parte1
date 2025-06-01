#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class MazeSolver:
    def __init__(self):
        # Suscriptor y publicador
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Distancias en el sector frontal, izquierdo y derecho
        self.front_dist = 10.0
        self.left_dist  = 10.0
        self.right_dist = 10.0

        # Umbrales para detección
        self.FRONT_THRESHOLD = 0.5
        self.SIDE_THRESHOLD  = 0.4

        # Velocidades
        self.linear_speed       = 0.15
        self.angular_speed_left = 0.4   # giro a la izquierda (+)
        self.angular_speed_right= -0.4  # giro a la derecha (−)

        # 90 grados = 1.57 rad aprox; a 0.4 rad/s tardamos ~3.9 s
        self.turn_duration_sec  = 4.0

        # Estados de la máquina
        # 0 => GO_STRAIGHT (hasta detectar la primera pared de frente)
        # 1 => TURN_RIGHT   (giro 90° la primera vez)
        # 2 => FOLLOW_LEFT_WALL
        self.state = 0
        self.turn_start_time = None

        # Frecuencia de control (10 Hz)
        self.rate = rospy.Rate(10)

    def get_sector_min_range(self, scan_msg, center_deg, half_width):
        """
        Devuelve la distancia mínima en el sector angular
        [center_deg - half_width, center_deg + half_width].
        """
        ranges = scan_msg.ranges
        n = len(ranges)
        if n < 1:
            return 10.0

        min_val = 10.0
        for angle in range(center_deg - half_width, center_deg + half_width + 1):
            idx = angle % 360
            r = ranges[idx]
            if r == 0.0 or r == float('inf'):
                r = 10.0
            if r < min_val:
                min_val = r
        return min_val

    def laser_callback(self, msg):
        """
        Leemos en 3 sectores:
         - Frente: ±30° alrededor de 0°
         - Izquierda: ±30° alrededor de 90°
         - Derecha: ±10° alrededor de 270°
        """
        self.front_dist = self.get_sector_min_range(msg, 0,   30)
        self.left_dist  = self.get_sector_min_range(msg, 30,  30)
        self.right_dist = self.get_sector_min_range(msg, 270, 10)

    def run(self):
        rospy.loginfo("Starting MazeSolver. State=0 => GO_STRAIGHT, then 1 => TURN_RIGHT, then 2 => FOLLOW_LEFT_WALL.")
        while not rospy.is_shutdown():
            twist = Twist()

            # Log de distancias y estado en cada iteración
            rospy.loginfo(
                f"[STATE={self.state}] "
                f"front={self.front_dist:.2f}, left={self.left_dist:.2f}, right={self.right_dist:.2f}"
            )

            # ============= STATE 0: GO_STRAIGHT =============
            if self.state == 0:
                # Avanza recto mientras front_dist >= FRONT_THRESHOLD
                if self.front_dist < self.FRONT_THRESHOLD:
                    # Detectamos primera pared delante => pasar a giro a la derecha
                    self.state = 1
                    self.turn_start_time = rospy.Time.now().to_sec()
                    rospy.loginfo("-> SWITCH to STATE 1: TURN_RIGHT")
                else:
                    # Avanzar
                    rospy.loginfo("GO_STRAIGHT: no pared enfrente, avanzando.")
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0

            # ============= STATE 1: TURN_RIGHT =============
            elif self.state == 1:
                current_time = rospy.Time.now().to_sec()
                elapsed = current_time - self.turn_start_time

                if elapsed < self.turn_duration_sec:
                    rospy.loginfo(f"TURN_RIGHT: turning for {elapsed:.1f}/{self.turn_duration_sec:.1f} sec.")
                    twist.linear.x  = 0.0
                    twist.angular.z = self.angular_speed_right
                else:
                    rospy.loginfo("-> SWITCH to STATE 2: FOLLOW_LEFT_WALL")
                    self.state = 2

            # ============= STATE 2: FOLLOW_LEFT_WALL =============
            else:  # self.state == 2
                # Caso 1) Pared delante Y pared izqda => giro derecha
                if (self.front_dist < self.FRONT_THRESHOLD) and (self.left_dist < self.SIDE_THRESHOLD):
                    rospy.loginfo("FOLLOW_LEFT_WALL: front & left blocked => turning RIGHT.")
                    twist.linear.x  = 0.0
                    twist.angular.z = self.angular_speed_right

                # Caso 2) Pared delante (pero no a la izqda) => giro izqda
                elif self.front_dist < self.FRONT_THRESHOLD:
                    rospy.loginfo("FOLLOW_LEFT_WALL: front blocked => turning LEFT.")
                    twist.linear.x  = 0.0
                    twist.angular.z = self.angular_speed_left

                # Caso 3) Izqda abierta => giro izqda
                elif self.left_dist > self.SIDE_THRESHOLD:
                    rospy.loginfo("FOLLOW_LEFT_WALL: left is free => turning LEFT.")
                    twist.linear.x  = 0.2                  # Avance lento
                    twist.angular.z = self.angular_speed_left + 0.2  # Giro moderado


                # Caso 4) Avanzar recto
                else:
                    rospy.loginfo("FOLLOW_LEFT_WALL: no front block, left has wall => going STRAIGHT.")
                    twist.linear.x  = self.linear_speed
                    twist.angular.z = 0.0

            # Publicamos
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

def main():
    rospy.init_node("maze_solver_tb3")
    solver = MazeSolver()
    solver.run()

if __name__ == "__main__":
    main()

