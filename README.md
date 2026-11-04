# MAQs
MAQs (Unitree Go2) conexión, control, documentación y automatización. 

## TODO
- [ ] Documentación de Conexión
- [ ] Limpieza del Doc.
- [ ] Automatización de 

## Guía de Conexión 

### Crear imagen Docker
```bash
docker compose up -d --build
```

### Activación Docker
- Uso no persistente: 
```bash
docker compose run --rm go2_dev
```

- Uso persistente:
```bash
docker compose start
```

Entrar a la terminal:
```bash
docker compose exec go2_dev zsh
```

Conexión SSH:
```bash
docker compose exec go2_dev maq_wire
```

> Es decir, el contendor se queda listo para su ejecución en todo momento.

Reinicio:
```bash
docker compose start
```

Pausa:
```bash
docker compose stop
```

Eliminación: 
```bash
docker compose down
```


### Conexión SSH - CPU Principal (Jetson)

```bash
maq_wire
```
> Alias para el comando: `ssh -X unitree@192.168.123.18`

```text
pssword: 123
```

### Inicializar Ros2
```bash 
init_ros2
```
> Alias para el comando: `source /workspace/maq-volume/unitree_ros2/setup.sh`


### Entorno ROS 2
```bash
ros:foxy(1) noetic(2) ?
> 1<CR>
```

### Ejecución de LiDAR

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

