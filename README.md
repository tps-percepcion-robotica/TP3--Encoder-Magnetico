# pierna_encoder_pkg

Simulación en ROS2 + RViz de una pierna de 2 articulaciones, donde cada articulación se
mueve con la lectura de un sensor angular conectado a un ESP32 con micro-ROS:

- `joint_cadera` —> potenciómetro (funcionando, conectado a RViz)
- `joint_rodilla` —> encoder magnético AS5600 (funcionando, conectado a RViz)

## Estado actual

| Fase | Descripción | Estado |
|---|---|---|
| 0 | Entorno ROS2 + workspace | ✅ |
| 1 | Modelo URDF de la pierna | ✅ |
| 2 | Visualización en RViz con `joint_state_publisher_gui` | ✅ |
| 3 | Calibración por YAML + nodo traductor (potenciómetro) | ✅ |
| 4 | Firmware ESP32 (potenciómetro + AS5600) + micro-ROS publicando en vivo | ✅ |
| 5 | Pierna en RViz movida por el potenciómetro real (cadera) | ✅ |
| 6 | Calibración + nodo traductor (AS5600) | ✅ |
| 7 | AS5600 conectado a `joint_rodilla` en RViz | ✅ |
| 8 | Comparación experimental potenciómetro vs. AS5600 | ⏳ pendiente |

## Estructura del repo

```
pierna_encoder_pkg/
├── urdf/
│   └── pierna.urdf.xacro          # 2 links (muslo, pierna) + 2 joints revolute
├── launch/
│   ├── display.launch.xml         # Simulación pura, sliders manuales (sin hardware)
│   ├── display_real.launch.xml    # RViz + robot_state_publisher + joint_state_publisher
│   │                               # (los nodos de calibración corren aparte)
│   └── pierna_hardware.launch.xml # Todo en uno: agente + ambos traductores + RViz
├── rviz/
│   └── pierna.rviz
├── config/
│   ├── calibracion_potenciometros.yaml  # offset_zero, raw_min/max, rango_grados
│   └── calibracion_as5600.yaml          # offset_grados, invertido, límites
├── pierna_encoder_pkg/
│   ├── calibracion_node.py           # /posicion (Int32) -> joint_cadera
│   ├── calibrar_potenciometro.py     # asistente de calibración (potenciómetro)
│   ├── calibracion_as5600_node.py    # /angulo (Float32) -> joint_rodilla
│   └── calibrar_as5600.py            # asistente de calibración (AS5600)
└── firmware/                      # Firmware ESP32 (ESP-IDF + micro-ROS)
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

`--symlink-install` es importante para que los YAML de calibración se actualicen sin recompilar.

## Firmware ESP32

### Conexión de pines

| Sensor | Señal | Pin ESP32 | Notas |
|---|---|---|---|
| Potenciómetro | Extremo A | 3.3V | Comparte rail con el AS5600 |
| Potenciómetro | Extremo B | GND | Comparte rail con el AS5600 |
| Potenciómetro | Cursor (medio) | **GPIO35** | Lectura directa de ADC, sin amplificador |
| AS5600 | VCC | 3.3V | **No usar 5V**: el cursor del pote y el AS5600 comparten lógica a 3.3V |
| AS5600 | GND | GND | |
| AS5600 | SDA | **GPIO21** | Cable naranja en el armado de referencia |
| AS5600 | SCL | **GPIO22** | Cable amarillo en el armado de referencia |

No hace falta un pin de 3.3V/GND por sensor: se puede compartir el mismo rail de la
protoboard para los dos.

### Tópicos publicados

| Tópico | Tipo | QoS | Sensor | Contenido |
|---|---|---|---|---|
| `/posicion` | `std_msgs/Int32` | Best effort | Potenciómetro | Posición 0–100 (%) |
| `/angulo` | `std_msgs/Float32` | Best effort | AS5600 | Ángulo absoluto 0–360°, calculado en el sensor |

`/voltaje` está deshabilitado: la librería micro-ROS que usamos (ver más abajo) solo
soporta 2 publishers a la vez, y priorizamos `/angulo` (AS5600) sobre `/voltaje`
(que no lo usa nada del lado ROS2).

Los dos tópicos usan QoS **best effort** a propósito: el ESP32 publica cada 20ms por
WiFi, y con QoS "reliable" (la que espera confirmación de cada mensaje) el buffer se
llena y todo se traba a ~1 mensaje por segundo. Los nodos de ROS2 que los escuchan
(`calibracion_node`, `calibrar_potenciometro`, `calibracion_as5600_node`,
`calibrar_as5600`) ya están configurados con la misma QoS — si escribís un nodo nuevo
que se suscriba a estos tópicos, tiene que usar `ReliabilityPolicy.BEST_EFFORT`
también, si no quedan incompatibles y no reciben nada.

### Compilar y flashear

**Instalación única del toolchain** (una sola vez por máquina):
```bash
~/.espressif/v5.5/esp-idf/install.sh esp32
```

**En cada terminal nueva** (antes de cualquier `idf.py`):
```bash
. ~/.espressif/v5.5/esp-idf/export.sh
cd ~/ros2_ws/src/pierna_encoder_pkg/firmware
```

**Componente micro-ROS** (una sola vez, en `firmware/components/`, no se versiona en git):
```bash
mkdir -p components && cd components
git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git
cd ..
```

> Nota interna del equipo: en esta máquina se reusa un `libmicroros.a` ya compilado
> de otro proyecto (evita compilar micro-ROS desde cero, que requiere un toolchain
> host completo con `catkin_pkg`/`empy`/`ament_cmake`, no siempre disponible). Esa
> librería cacheada tiene `RMW_UXRCE_MAX_PUBLISHERS=2`, por eso el firmware solo usa
> 2 publishers (`posicion` + `angulo`, sin `voltaje`). Si alguna vez hace falta un
> 3er publisher, hay que recompilar la librería desde una copia fresca del
> componente con `RMW_UXRCE_MAX_PUBLISHERS` más alto en su `colcon.meta` (o un
> `app-colcon.meta` propio en `firmware/`).

**Configurar WiFi y el Agente** (primera vez, o si cambia la red):
```bash
idf.py menuconfig
# micro-ROS Settings -> WiFi Configuration: SSID y Password
# micro-ROS Settings -> micro-ROS Agent IP: la IP de tu PC (ver `hostname -I`)
```

**Compilar y flashear:**
```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
(`/dev/ttyUSB0` puede variar — confirmar con `ls /dev/tty*` con la placa conectada.
Para salir del monitor: `Ctrl+]`.)

> Si el ESP32 no logra conectarse al agente después de flashear, apretá el botón
> **EN/RST** de la placa una vez que el agente (ver abajo) ya esté corriendo —
> a veces el primer intento de conexión pasa antes de que el agente esté listo y
> no se recupera solo.

## Uso: levantar todo el sistema

### Opción A — Simulación pura, sin sensores

```bash
ros2 launch pierna_encoder_pkg display.launch.xml
```
RViz + sliders manuales (`joint_state_publisher_gui`), para revisar que el modelo
articula bien sin hardware conectado.

### Opción B — Con los sensores reales, todo en una terminal

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

> Si ya tenés el agente corriendo en otra terminal, vas a ver `bind error ... errno:
> 98` (puerto ya usado) — es normal, dos agentes no pueden compartir el mismo
> puerto. En ese caso usá la Opción C.

### Opción C — Con los sensores reales, cada pieza en su propia terminal

Útil para debuggear paso a paso (ver qué tópico exactamente no está llegando).

**Terminal 1 — Agente micro-ROS** (siempre abierta)
```bash
cd ~/micro_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 — RViz** (siempre abierta)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pierna_encoder_pkg display_real.launch.xml
```

**Terminal 3 — Traductor potenciómetro (cadera)** (siempre abierta)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pierna_encoder_pkg calibracion_node --ros-args -p raw_topic:=/posicion
```

**Terminal 4 — Traductor AS5600 (rodilla)** (siempre abierta)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pierna_encoder_pkg calibracion_as5600_node
```

Si alguna de las Terminales 3 o 4 se cierra, el joint correspondiente se congela en
RViz en su última posición (no es un error, es que nadie le manda datos nuevos).

## Calibración

### Potenciómetro (cadera)

```bash
ros2 run pierna_encoder_pkg calibrar_potenciometro \
  --pot-id pierna_1 --raw-topic /posicion --rango-grados 180
```

Pide moverse a CERO / MÍNIMO / MÁXIMO (Enter en cada paso). Importante: mové la
**pierna armada** hasta sus topes físicos reales (no el eje del potenciómetro
suelto), y usá el `--rango-grados` real medido entre esos dos topes — si ponés un
valor distinto al real, el movimiento en RViz sale escalado mal (por ejemplo, se
mueve la mitad de lo que debería).

### AS5600 (rodilla)

```bash
ros2 run pierna_encoder_pkg calibrar_as5600 \
  --enc-id rodilla_1 --limite-max-grados 120
```

Más simple que el del potenciómetro: el AS5600 ya da grados reales, así que solo
hace falta el offset de cero y confirmar la dirección:
1. **CERO** — pierna recta (0°)
2. **Doblar un poco** la rodilla — con esto el script detecta solo si hay que
   invertir el signo

`--limite-max-grados` es el ángulo real al que llega la rodilla en su tope (en nuestro
armado, ~120°, cuando la pantorrilla toca el glúteo).

**En los dos casos:** después de calibrar, reiniciá el nodo traductor
correspondiente (Terminal 3 o 4, Ctrl+C y volver a correr) para que tome los valores
nuevos — los nodos solo leen el YAML una vez al arrancar.

## Créditos

- Firmware base de lectura de ADC, inspirado en
  [absss03/potenciometro](https://github.com/absss03/potenciometro)
- [micro-ROS](https://micro.ros.org/) / [micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component)
