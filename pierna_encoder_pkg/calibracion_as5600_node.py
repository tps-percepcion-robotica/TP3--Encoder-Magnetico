import math  #radianes
import os
import rclpy
import yaml  #leer calibración
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# publica /angulo (AS5600) "best effort", igual que /posicion.
QOS_SENSOR = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def normalizar_diferencia(diferencia_grados):
    """Lleva una diferencia de ángulos a [-180, 180) para manejar el
    'dado la vuelta' del AS5600 (ej: de 359° a 1° es +2°, no -358°)."""
    return (diferencia_grados + 180.0) % 360.0 - 180.0


class CalibracionAS5600(Node):

    def __init__(self):
        super().__init__('calibracion_as5600_node')

        default_config = os.path.join(
            get_package_share_directory('pierna_encoder_pkg'),
            'config', 'calibracion_as5600.yaml')

        self.declare_parameter('enc_id', 'rodilla_1')
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('angulo_topic', '/angulo')
        self.declare_parameter('angulo_rad_topic', '/rodilla_1/angulo_rad')
        self.declare_parameter('joint_name', 'joint_rodilla')
        self.declare_parameter('joint_states_topic', '/joint_states_sensores_rodilla')

        self.enc_id = self.get_parameter('enc_id').value
        self.config_path = self.get_parameter('config_path').value
        angulo_topic = self.get_parameter('angulo_topic').value
        angulo_rad_topic = self.get_parameter('angulo_rad_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        self.calib = self._cargar_calibracion(self.enc_id)

        self.pub = self.create_publisher(Float32, angulo_rad_topic, 10)
        self.joint_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.create_subscription(Float32, angulo_topic, self._on_angulo, QOS_SENSOR)

        self.get_logger().info(
            f"Calibración '{self.enc_id}' cargada desde {self.config_path}: {self.calib}")
        self.get_logger().info(
            f"Escuchando {angulo_topic} -> publicando {angulo_rad_topic} (rad) "
            f"y {joint_states_topic} (joint '{self.joint_name}')")

    def _cargar_calibracion(self, enc_id):
        with open(self.config_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        if enc_id not in data:
            raise RuntimeError(
                f"No hay calibración para '{enc_id}' en {self.config_path}. "
                'Corré calibrar_as5600.py primero.')
        return data[enc_id]

    def _on_angulo(self, msg: Float32):
        offset_grados = self.calib['offset_grados']
        invertido = self.calib['invertido']
        limite_min = self.calib['limite_min_grados']
        limite_max = self.calib['limite_max_grados']

        grados = normalizar_diferencia(msg.data - offset_grados)
        if invertido:
            grados = -grados

        grados_clamped = max(limite_min, min(limite_max, grados))
        angulo_rad = math.radians(grados_clamped)

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
    node = CalibracionAS5600()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
