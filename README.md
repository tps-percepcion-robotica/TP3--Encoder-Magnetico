# pierna_encoder_pkg

Simulación en ROS2 + RViz de una pierna de 2 articulaciones, donde cada articulación se
mueve con la lectura de un sensor angular conectado a un ESP32 con micro-ROS:

- `joint_cadera` —> potenciómetro
- `joint_rodilla` —> encoder magnético AS5600 

![Pierna armada](resource/WhatsApp%20Image%202026-09-04%20at%202.18.40%20PM%281%29.jpeg)

## Estructura del repo

```
pierna_encoder_pkg/
├── urdf/
│   └── pierna.urdf.xacro          # 2 links (muslo, pierna) + 2 joint
├── launch/
│   ├── display.launch.xml         # Simulación pura, sliders 
│   ├── display_real.launch.xml    # RViz + robot_state_publisher + joint_state_publisher
│   └── pierna_hardware.launch.xml # Todo en uno: agente + ambos traductores + RViz
├── rviz/
│   └── pierna.rviz
├── config/
│   ├── calibracion_potenciometros.yaml  # offset_zero, raw_min/max, rango_grados
│   └── calibracion_as5600.yaml          # offset_grados, invertido(mi caso), límites

├── pierna_encoder_pkg/
│   ├── calibracion_node.py           # /posicion (Int32) -> joint_cadera
│   ├── calibrar_potenciometro.py     # asistente de calibración (pote)
│   ├── calibracion_as5600_node.py    # /angulo (Float32) -> joint_rodilla
│   └── calibrar_as5600.py            # asistente de calibración (AS5600)
|
└── firmware/                      # Firmware ESP32 
    ├── main/
    │   ├── sensores_microros_main.c  # nodo micro-ROS: lee ambos sensores y publica
    │   ├── pote.c / pote.h           # lectura del potenciómetro (ADC directo)
    │   └── as5600.c / as5600.h       # lectura del AS5600 (I2C)
    └── pc_tools/
        └── potenciometro_monitor.py  # monitor de PC (posición, curva, log CSV)
```

## Compilar (lado ROS2)

```bash
cd ~/ros2_ws
colcon build --packages-select pierna_encoder_pkg --symlink-install
source install/setup.bash
```

`--symlink-install`!!!importante!!!

para que los YAML de calibración se actualicen sin recompilar.


## Firmware ESP32

### Conexión de pines

| Sensor | Señal | Pin ESP32 |
|---|---|---|---|
| Potenciómetro | Extremo A | 3.3V 
| Potenciómetro | Extremo B | GND 
| Potenciómetro | Cursor (medio) | **GPIO35** | Lectura directa de ADC, sin amplificador |


| AS5600 | VCC | 3.3V | 
| AS5600 | GND | GND | |
| AS5600 | SDA | **GPIO21** 
| AS5600 | SCL | **GPIO22** 


### Tópicos publicados

| Tópico | Tipo | QoS | Sensor | Contenido |
|---|---|---|---|---|
| `/posicion` | `std_msgs/Int32` | Best effort | Potenciómetro | Posición 0–100 (%) |
| `/angulo` | `std_msgs/Float32` | Best effort | AS5600 | Ángulo absoluto 0–360°, calculado en el sensor |



Los dos tópicos usan QoS **best effort** porqué: el ESP32 publica cada 20ms por
WiFi, y con QoS "reliable" (la que espera confirmación de cada mensaje) el buffer se
llena y todo se traba a ~1 mensaje por segundo!!!!

 Los nodos de ROS2 que los escuchan (`calibracion_node`, `calibrar_potenciometro`, `calibracion_as5600_node`,`calibrar_as5600`) ya están configurados con la misma QoS

LIMITACIÓN

`/voltaje` está deshabilitado: la librería micro-ROS que usamos (ver más abajo) solo
soporta 2 publishers a la vez, y priorizamos `/angulo` (AS5600) sobre `/voltaje`
(que no lo usa nada del lado ROS2).


## Uso


### Opción A — Simulación pura, sin sensores

```bash
ros2 launch pierna_encoder_pkg display.launch.xml
```
RViz + sliders manuales (`joint_state_publisher_gui`), para revisar que el modelo
articula bien sin hardware conectado.

### Opción B — Con los sensores , todo en una terminal

Con el ESP32 encendido y conectado a la misma red WiFi que la PC:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/micro_ws/install/setup.bash
source install/setup.bash
ros2 launch pierna_encoder_pkg pierna_hardware.launch.xml
```

Esto levanta junto: el agente micro-ROS (puerto UDP 8888), `robot_state_publisher`,
`calibracion_node` (cadera), `calibracion_as5600_node` (rodilla),
`joint_state_publisher` (mezcla los dos) y RViz.



**Terminal — Traductor potenciómetro (cadera)** (siempre abierta)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pierna_encoder_pkg calibracion_node --ros-args -p raw_topic:=/posicion
```

**Terminal — Traductor AS5600 (rodilla)** (siempre abierta)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pierna_encoder_pkg calibracion_as5600_node
```


## Calibración

### Potenciómetro (cadera)

```bash
ros2 run pierna_encoder_pkg calibrar_potenciometro \
  --pot-id pierna_1 --raw-topic /posicion --rango-grados 180
```

Pide moverse a CERO / MÍNIMO / MÁXIMO (Enter en cada paso). 

Importante: Se debe mover la **pierna armada** hasta sus topes físicos reales (ojo con el tope del pote) 

### AS5600 (rodilla)

```bash
ros2 run pierna_encoder_pkg calibrar_as5600 \
  --enc-id rodilla_1 --limite-max-grados 120
```

`--limite-max-grados` es el ángulo real al que llega la rodilla en su tope (en nuestro
armado, ~120°)


