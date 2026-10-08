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
docker run -it --rm \
  --name go2_dev \
  --network host \
  --privileged \
  -v ~/MAQs/maq-volume:/workspace/maq-volume \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=/run/user/1000 \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
  go2_ros2_humble # Docker MAQs
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

