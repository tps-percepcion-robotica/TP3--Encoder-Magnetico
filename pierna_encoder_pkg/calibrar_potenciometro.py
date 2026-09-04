#calibrar_potenciometro es un Script de calibración asistida (se corre UNA vez por potenciómetro)

#   ESP32(Int32 crudo)--> raw_topic--> ESTE SCRIPT (solo escucha) y escribe config/calibracion_potenciometros.yaml

#   ENTRADA : mensajes std_msgs/Int32 en raw_topic (el ADC crudo del ESP32).
#   SALIDA  : una entrada en el YAML bajo la clave pot_id,


import argparse
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32  # tipo de mensaje que manda el ESP32 (entero crudo del ADC)

# QoS del tópico de ENTRADA.
# El ESP32 publica "best effort"
QOS_SENSOR = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


#Nodo lector: se SUSCRIBE a raw_topic (best effort) y guarda la última lectura recibida
class LectorRaw(Node):

    def __init__(self, raw_topic):
        super().__init__('calibrador_potenciometro')  
        self.ultimo_valor = None
        # SUSCRIPCIÓN al tópico de entrada. Cada Int32 que llega dispara _on_msg
        self.create_subscription(Int32, raw_topic, self._on_msg, QOS_SENSOR)

    def _on_msg(self, msg):
        # Callback: se ejecuta en cada mensaje RECIBIDO. Solo copiamos el dato crudo
        self.ultimo_valor = msg.data


#Captura  
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
    # Clave bajo la que se guarda esta calibración en el YAML (calibracion_node.py
    # la busca con su parámetro pot_id).
    parser.add_argument('--pot-id', required=True, help='Identificador, ej: pierna_1')
    # Tópico de ENTRADA: de acá se leen los enteros crudos del ESP32.
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

    # -Captura de las 3 posiciones de referencia ---
    # Cada esperar_valor() BLOQUEA hasta recibir una lectura nueva del ESP32.
    print(f"=== Calibración de '{args.pot_id}' sobre {args.raw_topic} ===")
    # Raw en la posición que querés que valga 0°:
    offset_zero = esperar_valor(node, 'Mové el potenciómetro a la posición CERO de referencia')
    # Raw en el extremo mecánico mínimo:
    raw_min = esperar_valor(node, 'Mové al extremo MÍNIMO')
    # Raw en el extremo mecánico máximo:
    raw_max = esperar_valor(node, 'Mové al extremo MÁXIMO')

    # Ya capturamos todo: cerramos ROS, no hace falta seguir escuchando.
    node.destroy_node()
    rclpy.shutdown()


    # Si min y max son iguales, el span crudo es 0 y calibracion_node.py
    # dividiría por cero al calcular grados. Abortamos.
    if raw_min == raw_max:
        print('ERROR: raw_min y raw_max son iguales, no se puede calibrar. Abortando.')
        sys.exit(1)

    # --Entrada de calibración 
    nueva_entrada = {
        'offset_zero': int(offset_zero),   # raw que corresponde a 0°
        'raw_min': int(raw_min),           # raw en el extremo mínimo
        'raw_max': int(raw_max),           # raw en el extremo máximo
        'rango_grados': args.rango_grados, # grados totales entre raw_min y raw_max
    }

    # --- Escritura del YAML de SALIDA 
    if os.path.exists(args.config):
        with open(args.config, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
        os.makedirs(os.path.dirname(args.config), exist_ok=True)

    data[args.pot_id] = nueva_entrada  # agrega o reemplaza SOLO esta clave

    with open(args.config, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)

    print(f'\nCalibración guardada en {args.config}:')
    print(yaml.safe_dump({args.pot_id: nueva_entrada}, sort_keys=False))


if __name__ == '__main__':
    main()
