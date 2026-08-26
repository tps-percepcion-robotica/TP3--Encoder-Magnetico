import argparse
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Int32


class LectorRaw(Node):

    def __init__(self, raw_topic):
        super().__init__('calibrador_potenciometro')
        self.ultimo_valor = None
        self.create_subscription(Int32, raw_topic, self._on_msg, 1)

    def _on_msg(self, msg):
        self.ultimo_valor = msg.data


def esperar_valor(node, mensaje):
    input(f'\n{mensaje} y presioná Enter...')
    node.ultimo_valor = None
    while node.ultimo_valor is None:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node.ultimo_valor is None:
            print('  esperando datos en el tópico... ¿está corriendo el firmware/agent?')
    valor = node.ultimo_valor
    print(f'  -> capturado: {valor}')
    return valor


def main():
    parser = argparse.ArgumentParser(description='Calibración asistida de potenciómetro')
    parser.add_argument('--pot-id', required=True, help='Identificador, ej: pierna_1')
    parser.add_argument('--raw-topic', required=True,
                         help='Tópico Int32 crudo del ESP32, ej: /pierna_1/adc_raw')
    parser.add_argument('--rango-grados', type=float, required=True,
                         help='Rango mecánico total en grados entre MIN y MAX '
                              '(medido con transportador o del límite de diseño del joint)')
    parser.add_argument('--config', default=os.path.join(
        get_package_share_directory('pierna_encoder_pkg'), 'config',
        'calibracion_potenciometros.yaml'),
        help='Ruta al YAML de calibración (default: config instalado del paquete)')
    args = parser.parse_args()

    rclpy.init()
    node = LectorRaw(args.raw_topic)

    print(f"=== Calibración de '{args.pot_id}' sobre {args.raw_topic} ===")
    offset_zero = esperar_valor(node, 'Mové el potenciómetro a la posición CERO de referencia')
    raw_min = esperar_valor(node, 'Mové al extremo MÍNIMO')
    raw_max = esperar_valor(node, 'Mové al extremo MÁXIMO')

    node.destroy_node()
    rclpy.shutdown()

    if raw_min == raw_max:
        print('ERROR: raw_min y raw_max son iguales, no se puede calibrar. Abortando.')
        sys.exit(1)

    nueva_entrada = {
        'offset_zero': int(offset_zero),
        'raw_min': int(raw_min),
        'raw_max': int(raw_max),
        'rango_grados': args.rango_grados,
    }

    if os.path.exists(args.config):
        with open(args.config, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
        os.makedirs(os.path.dirname(args.config), exist_ok=True)

    data[args.pot_id] = nueva_entrada

    with open(args.config, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)

    print(f'\nCalibración guardada en {args.config}:')
    print(yaml.safe_dump({args.pot_id: nueva_entrada}, sort_keys=False))


if __name__ == '__main__':
    main()
