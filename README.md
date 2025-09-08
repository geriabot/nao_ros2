# nao_ros2
  
Antes de comenzar a utilizar este paquete es necesario seguir los siguientes pasos:
1. [Instalar Ubuntu 22.04 en el robot Nao](ubuntu.md)
2. [Configurar el robot Nao](conf.md)
3. [Instalar ROS 2 en el robot Nao](ros2.md)

## Instalación

Dependencias:
```bash
sudo apt-get install libmsgpack-dev
sudo apt-get install libignition-transport11-dev
pip install webrtcvad
pip install sounddevice
```

Compilación del paquete:
```bash
cd ~/nao_ws/src
git clone git@github.com:geriabot/nao_ros2.git
cd ~/nao_ws/
vcs import src < thirdparty.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

Como se puede observar, existen dependencias de paquetes externos, de los cuales hay que seguir las instrucciones de instalación.


**IMPORTANTE**: No compilar utilizando la opción `--symlink-install` para poder utilizar posteriormente `sync` para llevar el directorio /install al robot Nao.

---

En este punto, ya se encuentran compilados todos los paquetes necesarios. Ahora hay que transferir la compilación al robot **Nao**. Para ello, es necesario seguir los pasos del repositorio de Kenji Brameld con el objetivo de utilizar [sync](https://github.com/ijnek/sync).


```bash
cd ~/nao_ws/ 
./sync.sh nao <ip_nao>
```

Finalmente, ejecutar, tanto en el pc como en el nao:

```bash
source ~/nao_ws/install/setup.bash
```
---

**Recomendación:** Es probable que haya que ajustar el volumen de los micrófonos del robot **Nao** con *amixer* (recomendable ponerlos al 90%):

```bash
sudo apt update && sudo apt install alsa-utils
amixer set 'Numeric Left mics' 90% cap
amixer set 'Numeric Right mics' 90% cap
amixer set 'Analog Front mics' 90% cap
amixer set 'Analog Rear mics' 90% cap
``` 

## Puesta en marcha del robot Nao

Una vez encendido, ejecutar en el robot:

```
ros2 launch nao_ros2 nao.launch.py
```

Este comando activará:
* El **ModeSwitcher**, que gestiona el inicio y la detención de la locomoción del robot.
* La interacción por voz a través de los servidios `stt_service` y `tts_service`.
* El servidor de posiciones del robot `pos_action_server`.