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

| Topic | Tipo ROS 2 | Propósito |
| --- | --- | --- |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | Fuente principal de posición y orientación del cuerpo en `odom`; coloca el robot y permite acumular la nube en el mapa |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | Alternativa de posición y orientación si deja de llegar odometría reciente; necesita identificar el marco del cuerpo |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | Suscripción auxiliar que activa el aviso de falta de odometría; sus posiciones no se mezclan con el robot ni con el mapa |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | La misma suscripción auxiliar a baja frecuencia; no se usa para colocar el modelo |
| `/lowstate` | `unitree_go/msg/LowState` | Fuente prioritaria de los ángulos de los primeros 12 motores para animar las articulaciones de las patas |
| `/lf/lowstate` | `unitree_go/msg/LowState` | Alternativa a baja frecuencia cuando `/lowstate` deja de llegar durante un segundo; evita alternar ambas fuentes |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | Actualiza el indicador de giro alrededor del eje Z del LiDAR mediante `angular_velocity.z` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transformaciones dinámicas entre marcos, si se publican; el modo visual con `cloud_base` y odometría puede funcionar sin este topic |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Transformaciones fijas entre cuerpo y sensores, si se publican; permiten alinear la nube del sensor con el modelo |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | Escaneo en coordenadas del sensor, según `header.frame_id`; funciona en Solo LiDAR y necesita la transformación al cuerpo para superponerse al robot |
| `/utlidar/cloud_base` | `sensor_msgs/msg/PointCloud2` | Escaneo expresado respecto al cuerpo (`base_link` en el robot probado); fuente de Robot y entorno y del mapa acumulado local |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | Opción de nube compensada por movimiento; el visor respeta `header.frame_id` y requiere su transformación al mundo en Robot y entorno |
| `viewer/map` (opción del visor) | No es un topic ROS ni tiene un mensaje ROS propio | Mapa acumulado local creado en el navegador con `/utlidar/cloud_base` y odometría; vóxeles de 10 cm, hasta 80 000 puntos y máximo 5 actualizaciones por segundo |


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


## Fuentes

- [Official Go2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
- [Unitree ROS 2 definitions](https://github.com/unitreerobotics/unitree_ros2/tree/master/cyclonedds_ws/src/unitree/unitree_go/msg)
- [Unitree SDK PCL point profile](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk_pcl.h)
- [Unitree ROS 2 LiDAR publisher](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h)
- [ROS 2 common definitions](https://github.com/ros2/common_interfaces)
- [ROS bridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
- [Go2 viewer reference](https://robocloud-dashboard.vercel.app/urdf/go2)
