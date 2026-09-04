import argparse
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

QOS_SENSOR = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def normalizar_diferencia(diferencia_grados):
    return (diferencia_grados + 180.0) % 360.0 - 180.0


class LectorAngulo(Node):

    def __init__(self, angulo_topic):
        super().__init__('calibrador_as5600')
        self.ultimo_valor = None
        self.create_subscription(Float32, angulo_topic, self._on_msg, QOS_SENSOR)

    def _on_msg(self, msg):
        self.ultimo_valor = msg.data


def esperar_valor(node, mensaje):
    input(f'\n{mensaje} y presioná Enter...')
    node.ultimo_valor = None
    while node.ultimo_valor is None:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node.ultimo_valor is None:
            print('  esperando datos en el tópico... ¿está el AS5600 conectado y el ESP32 publicando?')
    valor = node.ultimo_valor
    print(f'  -> capturado: {valor:.2f}°')
    return valor


def main():
    parser = argparse.ArgumentParser(description='Calibración asistida de AS5600')
    parser.add_argument('--enc-id', required=True, help='Identificador, ej: rodilla_1')
    parser.add_argument('--angulo-topic', default='/angulo',
                         help='Tópico Float32 del AS5600 (grados), default /angulo')
    parser.add_argument('--limite-min-grados', type=float, default=0.0,
                         help='Límite inferior del rango mecánico, en grados (default 0)')
    parser.add_argument('--limite-max-grados', type=float, required=True,
                         help='Límite superior del rango mecánico, en grados '
                              '(ej: 120, cuando la pantorrilla toca el glúteo)')
    parser.add_argument('--config', default=os.path.join(
        get_package_share_directory('pierna_encoder_pkg'), 'config',
        'calibracion_as5600.yaml'),
        help='Ruta al YAML de calibración (default: config instalado del paquete)')
    args = parser.parse_args()

    rclpy.init()
    node = LectorAngulo(args.angulo_topic)

    print(f"=== Calibración de '{args.enc_id}' sobre {args.angulo_topic} ===")
    cero = esperar_valor(node, 'Poné la pierna en la posición CERO (recta) de referencia')
    print('\nAhora doblá un poco la rodilla (que aumente el ángulo real) y quedate quieto ahí.')
    prueba = esperar_valor(node, 'Con la rodilla doblada un poco')

    node.destroy_node()
    rclpy.shutdown()

    diferencia = normalizar_diferencia(prueba - cero)
    if abs(diferencia) < 0.5:
        print('ERROR: el ángulo no cambió entre los dos pasos. ¿Se movió la rodilla de verdad? '
              'Abortando sin guardar.')
        sys.exit(1)

    invertido = diferencia < 0
    if invertido:
        print('Detecté que doblar la rodilla BAJA el ángulo crudo -> se invierte el signo.')
    else:
        print('Detecté que doblar la rodilla SUBE el ángulo crudo -> signo normal.')

    nueva_entrada = {
        'offset_grados': float(cero),
        'invertido': invertido,
        'limite_min_grados': args.limite_min_grados,
        'limite_max_grados': args.limite_max_grados,
    }

    if os.path.exists(args.config):
        with open(args.config, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
        os.makedirs(os.path.dirname(args.config), exist_ok=True)

    data[args.enc_id] = nueva_entrada

    with open(args.config, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)

    print(f'\nCalibración guardada en {args.config}:')
    print(yaml.safe_dump({args.enc_id: nueva_entrada}, sort_keys=False))


if __name__ == '__main__':
    main()
