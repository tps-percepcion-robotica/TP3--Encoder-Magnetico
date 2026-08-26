#!/usr/bin/env python3
"""
Monitor para el potenciometro

- Muestra la posición actual (ticks) y velocidad (RPM) en la terminal.
- Grafica en vivo el historial de posición.

Uso:
    source /opt/ros/jazzy/setup.bash
    python3 potenciometro_monitor.py
"""
import csv
import math
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Import para ROS2
import rclpy
from rclpy.node import Node

# Dato usado por los topicos
from std_msgs.msg import Int32, Float32

CSV_PATH = "potenciometro_log.csv"
HISTORY_LEN = 500  # cantidad de puntos visibles en la curva

class PotenciometroMonitor(Node):
    def __init__(self):
        super().__init__('potenciometro_monitor')

        # Suscripciones
        self.create_subscription(Int32, '/posicion', self.posicion_cb, 10)
        self.create_subscription(Float32, '/voltaje', self.voltaje_cb, 10)

        self.current_porcentaje = 0.0
        self.current_angle_rad = 0.0
        self.last_voltage = 0.0
        self.t0 = time.time()

        self.time_hist = deque(maxlen=HISTORY_LEN)
        self.volt_hist = deque(maxlen=HISTORY_LEN)

        # CSV Logging
        self.csv_file = open(CSV_PATH, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'porcentaje', 'angulo_rad', 'voltaje'])
        self.get_logger().info(f"Logueando a {CSV_PATH}")

    def posicion_cb(self, msg):
        self.current_porcentaje = msg.data
        self.current_angle_rad = (self.current_porcentaje / 100.0) * (2 * math.pi)

    def voltaje_cb(self, msg):
        now = time.time()
        elapsed = now - self.t0
        self.last_voltage = msg.data

        print(f"[{elapsed:7.2f}s] Posición: {self.current_porcentaje:6.2f}% | Voltaje: {self.last_voltage:5.3f} V")

        self.time_hist.append(elapsed)
        self.volt_hist.append(msg.data)

        self.csv_writer.writerow([f"{now:.3f}", msg.data, f"{self.current_porcentaje:.2f}"            ])
        self.csv_file.flush()

    def destroy_node(self):
            self.csv_file.close()
            super().destroy_node()

def main():
    rclpy.init()
    node = PotenciometroMonitor()

    # Configuración de la gráfica
    fig, (ax_circle, ax_volt) = plt.subplots(1, 2, figsize=(11, 5))

    # --- Subplot 1: Círculo unitario ---
    unit_circle = plt.Circle((0, 0), 1.0, color='gray', fill=False, linestyle='--', linewidth=1.5)
    ax_circle.add_patch(unit_circle)
    vector_line, = ax_circle.plot([0, 1], [0, 0], color='crimson', linewidth=2.5, marker='o', label='Posición')
    
    ax_circle.set_xlim(-1.25, 1.25)
    ax_circle.set_ylim(-1.25, 1.25)
    ax_circle.set_aspect('equal')
    ax_circle.set_title("Posición Potenciómetro (0-100%)")
    ax_circle.axhline(0, color='lightgray', linewidth=0.8)
    ax_circle.axvline(0, color='lightgray', linewidth=0.8)
    ax_circle.grid(True, linestyle=':')

    # --- Subplot 2: Voltaje vs Tiempo ---
    volt_line, = ax_volt.plot([], [], color='royalblue', linewidth=1.8)
    ax_volt.set_xlabel("Tiempo (s)")
    ax_volt.set_ylabel("Voltaje (V)")
    ax_volt.set_title("Voltaje vs Tiempo")
    ax_volt.grid(True, linestyle=':')

    def update_plot(_frame):
        rclpy.spin_once(node, timeout_sec=0.01)

        # Calculo de coordenadas sobre el círculo unitario
        theta = node.current_angle_rad
        x = math.cos(theta)
        y = math.sin(theta)
        vector_line.set_data([0, x], [0, y])

        # Actualizar gráfica de voltaje
        if node.time_hist:
            volt_line.set_data(node.time_hist, node.volt_hist)
            ax_volt.relim()
            ax_volt.autoscale_view()

        return vector_line, volt_line

    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    
if __name__ == '__main__':
    main()