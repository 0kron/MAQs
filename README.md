# MAQs
MAQs (Unitree Go2) conexión, control, documentación y automatización. 

## TODO
- [ ] Documentación de Conexión
- [ ] Limpieza del Doc.
- [ ] Automatización de 

## Guía de Conexión 

Ubuntu 20.04LTS
Ubuntu 22.XLTS

### Activación Docker
```bash
# Al ejecutar el contenedor, montar el directorio
docker run -it --rm \
  --name go2_dev \
  --network host \
  -v ~/"{PWD}"/maq-volume:/workspace/maq-volume \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  go2_ros2_image
```

### Conexión SSH - CPU Principal (Jetson)

```bash
ssh -X unitree@192.168.123.18
```

```text
pssword: 123
```

### Entorno ROS 2
```bash
ros:foxy(1) noetic(2) ?
> 1 <CR>
```

### Ejecución de LiDAR

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```


## Trouble Shooting
# Dependencias y Software de MAQs
