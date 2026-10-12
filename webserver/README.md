# Demo datos en vivo Go2

## Archivos y flujo de datos
- `robot.py`: Genera datos sinteticos de topics para verificar que podemos leer topics y mostrarlos en vivo en la pagina web.
Genera datos del movimiento del robot, kinematica inversa de las piernas e impactos del rayo de LiDAR. 
- `main.html`: Define la estrucutra de la pagina y carga el modulo de java script.  
- `main.js`: Da el comportamiento de la pagina. Usa Three.js y el loader de URDF para generar la escena 3D y renderizar el robot. Se conecta a ROS mediante el WebSocker y actualiza la pagina con los datos entrantes. 
- `pointcloud.mjs`: Nos ayuda procesar los datos de la nube de puntos del LiDAR. Sus dos funciones se usan en `main.js`. La función `decodeCloud(message)` interpreta el mensaje ROS con los datos de los puntos en binario y extrae las coordinadas (x, y, z), tambien extrae los valores de intensidad si estan disponibles. A lo maximo samplea 100,000 puntos para un renderizado eficiente. La función `intensityColors(intensities, range)` convierte los valores de intensidad en colores, azul para baja intensidad hasta rojo para alta intensidad. Usa gris si no esta disponible el valor de intensidad. La extensión `.mjs` significa que es un modulo de JavaScript, y puede exportar funciones (En JS para importar una función, hay que declarar que se va a exportar primero). 
- `assets`:  Aqui esta el URDF (Unified robot description format) y archivos .dae para el modelo 3d del robot. 
- `vendor/`: Aqui se encuentra la libreria three.js y el loader de URDF

El flujo se puede ver como: 
`robot.py → rosbridge publish JSON → main.js → robot body/joints + LiDAR points`

rosbridge actua como una conexión entre el sistema ROS y nuestro browser. Los nodos ROS del robot publican datos mediante topicos. rosbridge hace que estos datos esten disponibles en una conexión WebSocket. En este proyecto `main.js` es el encargado de conectarse a ese WebSocket y suscribirse a los topics. 


## Topics que usamos

| Topic | ROS 2 type | Propósito |
| --- | --- | --- |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | Posición, orientación y velocidad del cuerpo |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | Fuente alternativa de posición y orientación del cuerpo |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | IMU del cuerpo, patas y estado de movimiento |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | Estado de movimiento de baja frecuencia |
| `/lowstate` | `unitree_go/msg/LowState` | Estructura completa de 20 motores; los primeros 12 animan las patas |
| `/lf/lowstate` | `unitree_go/msg/LowState` | Estado de los motores de baja frecuencia |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | Orientación, giroscopio y aceleración del LiDAR |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom → base` y articulaciones móviles del URDF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Transformaciones fijas del URDF, incluida `base → radar` |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | Escaneo en vivo en `radar` |
| `/utlidar/cloud_base` | `sensor_msgs/msg/PointCloud2` | El mismo escaneo en el sistema de referencia del cuerpo |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | El mismo escaneo instantáneo en `odom` |
| `/uslam/cloud_map`, `/utlidar/voxel_map` | `sensor_msgs/msg/PointCloud2` | Puntos acumulados de los escaneos, con filtrado por vóxeles de 10 cm |
| `/utlidar/lidar_state` | `unitree_go/msg/LidarState` | Estado sintético del sensor |



## Tipos de nubes de puntos y sus diferencias
Las principales diferencias entre estos topics y formas de ver las nubes de puntos, son la etapa que visualizamos y el sistema de coordenadas que usamos. 
- `utlidar/cloud`: Los puntos que detecta actualmente el LiDAR. Medidos desde el sensor de LiDAR como punto de referencia.
- `utlidar/cloud_base`: Mismos puntos pero ahora con la base del robot como referencia. 
- `utlidar/cloud_deskewed`: Puntos corregidos por movimiento. Desde que el LiDAR recolecto los datos, el robot se pudo haber movido, deskewed (desviado) es lo que compensa el movimiento del robot. 
- `uslam/cloud_map`: Puntos registrados en el mapa SLAM (Simultaneous localization and mapping), los puntos que ha registrado el robot los registramos y acumulamos en un mapa mediante el robot se mueve en el. 
- `utlidar/voxel_map`: El mapa de puntos acumulados pero ahora representado mediante voxeles. Los voxeles son cubos 3D, se usan para representar el espacio 3D de forma mas discreta. 

## Telemetría real
Para leer telemetría real con el robot, hay que poner la dirección de rosbridge de nuestro robot en un WebSocker. En nuestro caso: 
`ws://192.168.123.18:9090`.

Si el robot no publica `/tf` o `/tf_static`, selecciona **Vista → Solo LiDAR (sin TF)**.
Esta vista muestra los puntos y su intensidad en las coordenadas originales del mensaje y oculta el robot y el suelo. Para superponer la nube con el modelo hace falta la transformación real del sensor; no se supone que `base`, `base_link`, `radar` y `utlidar_lidar` sean equivalentes. El modo **Robot y entorno** sigue usando las transformaciones recibidas.

En **Robot y entorno**, **Referencia del modelo → Cuerpo de odometría (visual)** coloca la raíz visual del cuerpo del URDF en el `child_frame_id` de `/utlidar/robot_odom` cuando falta una TF entre ese marco y `base`. Es una configuración visual explícita con desplazamiento cero entre ambos orígenes, no una calibración medida ni una TF añadida a ROS. Para exigir exclusivamente la transformación calibrada, elige **Base del URDF (TF)**. El modelo se oculta si no se puede colocar o si la posición deja de llegar. No se mezclan las posiciones de `sportmodestate`, que no declaran marcos, con las de odometría.

Para ver puntos junto al robot sin la TF del sensor, elige **`/utlidar/cloud_base`**: el visor usa el `frame_id` real y la odometría que declara ese mismo cuerpo; no vuelve a aplicar el montaje del LiDAR a una nube ya transformada por el robot. La alineación física exacta de la raíz visual con el cuerpo debe contrastarse con la descripción/calibración de la unidad real.

**Mapa acumulado local (cloud_base)** acumula en el navegador la nube `/utlidar/cloud_base` usando la transformación de la odometría al instante del escaneo. Conserva un punto por vóxel de 10 cm, limita el mapa a 80 000 puntos y actualiza como máximo a 5 Hz. Requiere odometría reciente y una transformación válida; no acumula coordenadas del sensor como si fueran coordenadas del mundo. **Limpiar mapa local** empieza un mapa nuevo. Cambiar de marco global o reconectar borra el mapa; si el robot reinicia su odometría sin cambiar el nombre del marco, limpia el mapa manualmente. Este mapa dura durante la sesión del navegador y no procede del SLAM del robot.

La cuadrícula se coloca inicialmente bajo el pie más bajo del URDF, incluyendo el radio de 22 mm del pie y una separación visual de 3 mm. Después permanece fija en el marco global: no sigue al cuerpo ni sube y baja con cada paso. **Ajustar piso a patas** permite establecer de nuevo esa referencia. Es una referencia visual basada en las articulaciones; no mide ni representa terreno real.

El dibujo del cuerpo y las articulaciones se suaviza entre mensajes con una constante de 55 ms. Se prioriza `/lowstate` y se usa `/lf/lowstate` si deja de llegar la primera fuente, evitando alternar muestras de ambas. La nube y el mapa siguen usando la odometría original, sin este suavizado visual.

En **Robot y entorno**, la cámara gira alrededor del cuerpo y acompaña su desplazamiento, conservando el ángulo, el zoom y los ajustes manuales. **Restablecer vista** vuelve a centrarla cerca del robot, también al ver mapas acumulados. En **Solo LiDAR** o cuando no se puede colocar el modelo, la vista del mapa se ajusta a la extensión de los puntos.

Por ahora, el menú ofrece únicamente **Mapa acumulado local (cloud_base)** como mapa, seleccionado de forma predeterminada. `/uslam/cloud_map` y `/utlidar/voxel_map` se retiraron del menú porque no se recibieron mensajes ni en el visor ni en las pruebas de la terminal del robot. El soporte interno permanece para poder habilitarlos cuando estén disponibles.

El simulador ahora inicia a media velocidad. Ejecuta `python3 robot.py --host 0.0.0.0 --speed 0.5`; usa `--speed 1` para la velocidad original o `--speed 0` para pausar el movimiento. La frecuencia de entrega de mensajes permanece igual. Esto afecta únicamente a los datos sintéticos de `robot.py`, no al robot real conectado por rosbridge.



## Fuentes

- [Official Go2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
- [Unitree ROS 2 definitions](https://github.com/unitreerobotics/unitree_ros2/tree/master/cyclonedds_ws/src/unitree/unitree_go/msg)
- [Unitree SDK PCL point profile](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk_pcl.h)
- [Unitree ROS 2 LiDAR publisher](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h)
- [ROS 2 common definitions](https://github.com/ros2/common_interfaces)
- [ROS bridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
- [Go2 viewer reference](https://robocloud-dashboard.vercel.app/urdf/go2)
