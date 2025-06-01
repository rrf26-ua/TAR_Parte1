#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading

x_positions = []
y_positions = []
# Bloqueo para el acceso concurrente
data_lock = threading.Lock()

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo("Robot Position: x={:.2f}, y={:.2f}".format(x, y))
    with data_lock:
        x_positions.append(x)
        y_positions.append(y)

def live_plot():
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], marker='o', linestyle='-')
    ax.set_xlabel("Posición X (m)")
    ax.set_ylabel("Posición Y (m)")
    ax.set_title("Trayectoria del Robot (Live)")
    ax.grid(True)
    
    while not rospy.is_shutdown():
        # Copiar los datos de forma segura
        with data_lock:
            xs = list(x_positions)
            ys = list(y_positions)
        
        # Actualiza los datos del gráfico
        line.set_data(xs, ys)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.1)  # Pequeña pausa para permitir la actualización

def main():
    rospy.init_node("dibuja_mov", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    plot_thread = threading.Thread(target=live_plot)
    plot_thread.daemon = True
    plot_thread.start()
    rospy.spin()
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()

