import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32


class CalibracionPotenciometro(Node):

    def __init__(self):
        super().__init__('calibracion_potenciometro_node')

        default_config = os.path.join(
            get_package_share_directory('pierna_encoder_pkg'),
            'config', 'calibracion_potenciometros.yaml')

        self.declare_parameter('pot_id', 'pierna_1')
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('raw_topic', '/pierna_1/adc_raw')
        self.declare_parameter('angulo_topic', '/pierna_1/angulo_rad')
        self.declare_parameter('joint_name', 'joint_cadera')
        self.declare_parameter('joint_states_topic', '/joint_states_sensores')

        self.pot_id = self.get_parameter('pot_id').value
        self.config_path = self.get_parameter('config_path').value
        raw_topic = self.get_parameter('raw_topic').value
        angulo_topic = self.get_parameter('angulo_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        self.calib = self._cargar_calibracion(self.pot_id)

        self.pub = self.create_publisher(Float32, angulo_topic, 10)
        self.joint_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.create_subscription(Int32, raw_topic, self._on_raw, 10)

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
        raw_min = self.calib['raw_min']
        raw_max = self.calib['raw_max']
        offset_zero = self.calib['offset_zero']
        rango_grados = self.calib['rango_grados']

        lo, hi = sorted((raw_min, raw_max))
        raw_clamped = max(lo, min(hi, msg.data))

        grados = (raw_clamped - offset_zero) * (rango_grados / (raw_max - raw_min))

        angulo_rad = math.radians(grados)

        salida = Float32()
        salida.data = angulo_rad
        self.pub.publish(salida)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [self.joint_name]
        joint_msg.position = [angulo_rad]
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
