# pierna_encoder_pkg

Simulación en ROS2 + RViz de una pierna de 2 articulaciones, donde cada articulación se
mueve con la lectura de un sensor angular  conectado a un ESP32 con micro-ROS

- joint_cadera—> potenciómetro (implementado y funcionando)
- joint_rodilla`—> encoder magnético AS5600 (pendiente)

## Estado actual

| Fase | Descripción | Estado |
|---|---|---|
| 0 | Entorno ROS2 + workspace
| 1 | Modelo URDF de la pierna      
| 2 | Visualización en RViz con `joint_state_publisher_gui`    
| 3 | Calibración por YAML + nodo traductor crudo→ángulo 
| 4 | Firmware ESP32 (potenciómetro) + micro-ROS publicando en vivo 
| 5 | Pierna en RViz movida por el potenciómetro real  (hasta aca llegué)
| 6 | Firmware + integración del AS5600 (rodilla) 
| 7 | Comparación experimental potenciómetro vs. AS5600 

Mientras no esté el AS5600, `joint_rodilla` se muestra en RViz con un valor fijo por
defecto (no es un error)

## Estructura del repo

```
pierna_encoder_pkg/
├── urdf/
│   └── pierna.urdf.xacro          # 2 links (muslo, pierna) + 2 joints revolute
├── launch/
│   ├── display.launch.xml         # acá tora sliders manuales 
│   ├── display_real.launch.xml    # RViz + robot_state_publisher + joint_state_publisher
│   │                               # (calibracion_node corriendo aparte)
│   └── pierna_hardware.launch.xml # Todo en uno: agente micro-ROS + calibración + RViz
├── rviz/
│   └── pierna.rviz                
├── config/
│   └── calibracion_potenciometros.yaml  # Calibración por potenciómetro (offset_zero, raw_min/max, rango_grados)
├── pierna_encoder_pkg/
│   ├── calibracion_node.py        # Nodo: crudo (Int32) -> ángulo calibrado -> /joint_states
│   └── calibrar_potenciometro.py  # Script asistido de calibración 
└── firmware/
    └── potenciometro_esp32/       # Firmware ESP32 (ESP-IDF + micro-ROS), ver crédito abajo
```


## Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select pierna_encoder_pkg --symlink-install
source install/setup.bash
```


## Uso

### Opción A — Simulación pura, sin sensores (mover con sliders)

Para revisar que el modelo articula bien, sin hardware conectado:

```bash
ros2 launch pierna_encoder_pkg display.launch.xml
```

Abre RViz + una ventana con sliders

### Opción B — Con el potenciómetro real 

Con el ESP32 encendido y conectado a la misma red WiFi que la PC:

```bash
# sourcear también el workspace donde está compilado micro_ros_agent
source ~/micro_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch pierna_encoder_pkg pierna_hardware.launch.xml
```

Esto levanta: `micro_ros_agent` (puerto UDP 8888) + `robot_state_publisher` +
`calibracion_node` + `joint_state_publisher` (rellena `joint_rodilla` con un valor
por defecto) + RViz.


## Firmware ESP32

`firmware/potenciometro_esp32/` es una copia local del repo de abi!!!
 lee el potenciómetro
por ADC (`esp_adc/adc_oneshot`) y publica por micro-ROS:

| Tópico | Tipo | Contenido |
|---|---|---|
| `/posicion` | `std_msgs/Int32` | Posición 0–100 (%) |
| `/voltaje` | `std_msgs/Float32` | Voltaje de salida 0–3.3V |

Compilar y flashear (con el ESP-IDF activado, `. $IDF_PATH/export.sh`):

```bash
cd firmware/potenciometro_esp32
mkdir -p components && cd components
git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git
cd ..
idf.py menuconfig   # micro-ROS Settings: Agent IP/Port, WiFi SSID/Password
idf.py build
idf.py flash monitor
```
