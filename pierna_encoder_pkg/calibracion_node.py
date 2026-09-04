#calibracion_node es un NODO ROS2 que corre siempre mientras usás la pierna

# Agarra el número crudo del ADC que manda el ESP32 y lo transforma en un ángul físico para RViz / robot_state_publisher

#   ENTRADA : std_msgs/Int32 en raw_topic (ADC crudo del ESP32).
#   ENTRADA :  calibracion_potenciometros.yaml (lo genera calibrar_potenciometro.py).
#   SALIDA  :  - std_msgs/Float32 en angulo_topic  -> ángulo en radianes (para debug/graficar).
#              - sensor_msgs/JointState en joint_states_topic -> mueve el joint del URDF.

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState  # mensaje de SALIDA que consume robot_state_publisher
from std_msgs.msg import Float32, Int32  # Int32 = ENTRADA cruda, Float32 = SALIDA en rad


QOS_SENSOR = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class CalibracionPotenciometro(Node):

    def __init__(self):
        super().__init__('calibracion_potenciometro_node')  

        # Ruta por defecto del YAML
        default_config = os.path.join(
            get_package_share_directory('pierna_encoder_pkg'),
            'config', 'calibracion_potenciometros.yaml')

        self.declare_parameter('pot_id', 'pierna_1')                  # qué entrada del YAML usar
        self.declare_parameter('config_path', default_config)         # ruta del YAML
        self.declare_parameter('raw_topic', '/pierna_1/adc_raw')      # tópico de ENTRADA (crudo)
        self.declare_parameter('angulo_topic', '/pierna_1/angulo_rad')          # tópico de SALIDA (Float32 rad)
        self.declare_parameter('joint_name', 'joint_cadera')                    # joint del URDF a mover
        self.declare_parameter('joint_states_topic', '/joint_states_sensores')  # tópico de SALIDA (JointState)

        # Leemos los valores efectivos de cada parámetro
        self.pot_id = self.get_parameter('pot_id').value
        self.config_path = self.get_parameter('config_path').value
        raw_topic = self.get_parameter('raw_topic').value
        angulo_topic = self.get_parameter('angulo_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        # Cargamos del YAML el dict {offset_zero, raw_min, raw_max, rango_grados} de este pot_id.
        self.calib = self._cargar_calibracion(self.pot_id)

        #ángulo suelto en radianes

        self.pub = self.create_publisher(Float32, angulo_topic, 10)
        
        # JointState lo que escucha robot_state_publisher para mover el modelo en RViz.
        self.joint_pub = self.create_publisher(JointState, joint_states_topic, 10)

        
        # Cada Int32 crudo que llega del ESP32 dispara _on_raw.
        self.create_subscription(Int32, raw_topic, self._on_raw, QOS_SENSOR)

        self.get_logger().info(
            f"Calibración '{self.pot_id}' cargada desde {self.config_path}: {self.calib}")
        self.get_logger().info(
            f"Escuchando {raw_topic} -> publicando {angulo_topic} (rad) "
            f"y {joint_states_topic} (joint '{self.joint_name}')")

    def _cargar_calibracion(self, pot_id):
        with open(self.config_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        if pot_id not in data:
            raise RuntimeError(
                f"No hay calibración para '{pot_id}' en {self.config_path}. "
                'Corré calibrar_potenciometro.py primero.')
        return data[pot_id]

    def _on_raw(self, msg: Int32):
        # CALLBACK  
        # Convierte crudo -> ángulo y publica en los dos tópicos de salida.
        raw_min = self.calib['raw_min']
        raw_max = self.calib['raw_max']
        offset_zero = self.calib['offset_zero']
        rango_grados = self.calib['rango_grados']

        lo, hi = sorted((raw_min, raw_max))
        raw_clamped = max(lo, min(hi, msg.data))

        # FÓRMULA de conversión:
        #   rango_grados / (raw_max - raw_min) = escala en grados por cuenta de ADC
        #   (raw_clamped - offset_zero)        = cuántas cuentas estás del CERO de referencia
        grados = (raw_clamped - offset_zero) * (rango_grados / (raw_max - raw_min))

        angulo_rad = math.radians(grados)

        # ---  Float32 con el ángulo 
        salida = Float32()
        salida.data = angulo_rad
        self.pub.publish(salida)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()  # timestamp, necesario para TF
        joint_msg.name = [self.joint_name]                        # qué joint del URDF
        joint_msg.position = [angulo_rad]                         # en qué ángulo ponerlo
        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibracionPotenciometro()
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
