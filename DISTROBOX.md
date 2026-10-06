# Compilar nao_ws en distrobox (Ubuntu 22.04 + ROS 2 Humble)

El robot Nao usa Ubuntu 22.04 con ROS 2 Humble instalado por `apt`. Lo que se le envía con
`sync.sh` (el directorio `install/`) tiene que compilarse en ese mismo sistema: mismas
versiones de Python (3.10), Boost, numpy y paquetes `ros-humble-*`. Si el ordenador no tiene
Ubuntu 22.04, la forma más sencilla es un contenedor [distrobox](https://distrobox.it/):
comparte el `$HOME`, la red, el sonido y la pantalla con el ordenador, pero por dentro es
una Ubuntu 22.04 real.

> Entornos como pixi/RoboStack **no sirven** para compilar lo que va al robot: usan otras
> versiones de Python y de las librerías, y los binarios no funcionan en el Nao.

## 1. Crear el contenedor

En el ordenador (necesita `distrobox` y `podman` o `docker`; en Ubuntu: `sudo apt install distrobox podman`):

```bash
distrobox create -n ubuntu22 -i docker.io/library/ubuntu:22.04
distrobox enter ubuntu22
```

Todo lo que sigue se hace **dentro** del contenedor (el prompt cambia a `usuario@ubuntu22`).

## 2. Instalar ROS 2 Humble

```bash
sudo apt update
sudo apt install -y software-properties-common curl locales
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools python3-pip ros-humble-rmw-cyclonedds-cpp
sudo rosdep init
rosdep update
```

`ros-humble-rmw-cyclonedds-cpp` hace falta si en tu `~/.bashrc` tienes
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: el contenedor hereda las variables de entorno
del ordenador y, sin ese paquete, la compilación falla con
`Could not find ROS middleware implementation 'rmw_cyclonedds_cpp'`.

Para no tener que cargar ROS a mano cada vez, solo dentro del contenedor (el `~/.bashrc` es
el mismo que el del ordenador, por eso se comprueba `CONTAINER_ID`):

```bash
cat >> ~/.bashrc <<'EOF'
if [ "$CONTAINER_ID" = "ubuntu22" ]; then
  source /opt/ros/humble/setup.bash
  [ -f ~/nao_ws/install/setup.bash ] && source ~/nao_ws/install/setup.bash
fi
EOF
```

(Ajusta `~/nao_ws` si el workspace está en otra ruta.)

## 3. Descargar y compilar el workspace

```bash
mkdir -p ~/nao_ws/src && cd ~/nao_ws/src
git clone git@github.com:geriabot/nao_ros2.git
cd ~/nao_ws
vcs import src < src/nao_ros2/thirdparty.repos
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y libignition-transport11-dev   # walk_bot (no lo declara en package.xml)
source /opt/ros/humble/setup.bash
colcon build
```

No uses `--symlink-install`: `sync.sh` copia `install/` al robot y los enlaces simbólicos
no llegan. Ojo con `~/.colcon/defaults.yaml`: como el `$HOME` es compartido, si ahí tienes
`symlink-install: true` se aplica también dentro del contenedor; quítalo de ese fichero.

### nao_meshes y su licencia

`nao_meshes` descarga las mallas del Nao y, al compilarlo, abre un diálogo para aceptar
su licencia. Esto pasa **en cada** `colcon build` que lo incluya, así que lo práctico es
compilarlo una vez (aceptando el diálogo) y omitirlo después. La primera compilación tiene
que incluirlo, porque `nao_description` depende de él:

```bash
colcon build                              # la primera vez: acepta la licencia
colcon build --packages-skip nao_meshes   # las siguientes
```

Si se compila sin pantalla (por SSH, por ejemplo), se puede aceptar la licencia sin diálogo
con `I_AGREE_TO_NAO_MESHES_LICENSE=1 colcon build`.

## 4. Llevar la compilación al Nao

Desde el contenedor, igual que en el [README](README.md#transferir-la-compilación-al-nao)
(la primera vez hay que descargar `sync.sh` en la raíz del workspace):

```bash
cd ~/nao_ws
curl -O https://raw.githubusercontent.com/ijnek/sync/main/sync.sh && chmod +x sync.sh   # solo la primera vez
./sync.sh nao <ip_nao>
```

## 5. Simulación con Webots (opcional)

El controlador del Nao en Webots (`nao_webots/controllers/nao_lola_python`) usa `rclpy` y
`cv_bridge`, así que Webots tiene que instalarse y lanzarse **dentro** del contenedor, con
ROS cargado (OpenCV y numpy, que también usa, ya vienen con `ros-humble-desktop`):

```bash
curl -L -o /tmp/webots.deb https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install -y /tmp/webots.deb
```

Para usarlo:

```bash
webots ~/nao_ws/src/nao_webots/worlds/nao_indoors.wbt   # terminal 1
ros2 launch nao_ros2 simnao.launch.py                     # terminal 2
```

Los servicios de voz de `simple_hri` (`stt_service`, `tts_service`) necesitan además las
dependencias y credenciales descritas en [Dependencias en el Nao](README.md#dependencias-en-el-nao).

## Uso diario

```bash
distrobox enter ubuntu22
cd ~/nao_ws
colcon build --packages-skip nao_meshes
```

Para borrar el contenedor: `distrobox rm ubuntu22` (el workspace, que está en tu `$HOME`,
no se toca).
