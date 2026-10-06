# nao_ros2
  
Antes de comenzar a utilizar este paquete es necesario seguir los siguientes pasos:
1. [Instalar Ubuntu 22.04 en el robot Nao](ubuntu.md)
2. [Configurar el robot Nao](conf.md)
3. [Instalar ROS 2 Humble en el robot Nao y en el ordenador](ros2.md)
4. Tener Ubuntu 22.04 en el ordenador (o virtualizarlo). Si tu ordenador usa otra distribución, puedes compilar en un contenedor: [instrucciones con distrobox](DISTROBOX.md)

## Instalación

Todo se compila en el **ordenador** (Ubuntu 22.04 + ROS 2 Humble) y después se copia al robot.

### Compilación en el ordenador

```bash
mkdir -p ~/nao_ws/src && cd ~/nao_ws/src
git clone git@github.com:geriabot/nao_ros2.git
cd ~/nao_ws/
vcs import src < src/nao_ros2/thirdparty.repos
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libignition-transport11-dev   # necesario para compilar walk_bot (no está en su package.xml)
colcon build
```

`thirdparty.repos` descarga los paquetes externos (locomoción, LoLa, LEDs, posturas, interacción por voz con [simple_hri](https://github.com/rodperex/simple_hri)...) y también los mundos de Webots ([nao_webots](https://github.com/rodperex/nao_webots)) y [nao_pos_recorder](https://github.com/rodperex/nao_pos_recorder). Algunos de estos paquetes externos tienen sus propias instrucciones de instalación, que también hay que seguir.

**IMPORTANTE**: No compilar utilizando la opción `--symlink-install` (revisa también que no esté activada en `~/.colcon/defaults.yaml`): `sync.sh` copia el directorio `install/` al robot y los enlaces simbólicos no llegan.

El paquete `nao_meshes` descarga las mallas del Nao y, al compilarlo, abre un diálogo para aceptar su licencia. Ocurre en cada compilación que lo incluya, así que tras la primera se puede omitir con `colcon build --packages-skip nao_meshes`.

### Transferir la compilación al Nao

Para llevar el directorio `install/` al robot se utiliza el script `sync.sh` del repositorio de Kenji Brameld [ijnek/sync](https://github.com/ijnek/sync) (en el robot tiene que estar instalado `rsync`: `sudo apt install rsync`). Descárgalo en la raíz del workspace:

```bash
cd ~/nao_ws/
curl -O https://raw.githubusercontent.com/ijnek/sync/main/sync.sh
chmod +x sync.sh
```

Y cada vez que quieras actualizar el robot:

```bash
cd ~/nao_ws/
./sync.sh nao <ip_nao>
```

Además de copiar `install/`, `sync.sh` instala en el robot (con `apt`) las dependencias de ejecución que declaran los paquetes en su `package.xml`.

### Dependencias en el Nao

Lo que no se instala automáticamente con `sync.sh`:

```bash
sudo apt-get install libmsgpack-dev
sudo apt-get install libignition-transport11-dev
sudo apt-get install alsa-utils libportaudio2   # aplay/amixer (audio) y PortAudio (micrófonos)
pip install webrtcvad
pip install sounddevice
pip install tf_transformations # needed to have odometry
pip install openai google-cloud-texttospeech
```

`alsa-utils`, `libportaudio2`, `webrtcvad`, `sounddevice`, `openai` y `google-cloud-texttospeech` son las dependencias de los servicios de voz de [simple_hri](https://github.com/rodperex/simple_hri) en su versión en la nube (`stt_service` con la API de OpenAI y `tts_service` con Google Cloud TTS), que es la que lanza `nao.launch.py`. Estos servicios necesitan además las credenciales `OPENAI_API_KEY` y `GOOGLE_APPLICATION_CREDENTIALS`: ver [cómo configurarlas en simple_hri](https://github.com/rodperex/simple_hri#4-configure-api-credentials).

### Carga del entorno

Tanto en el ordenador como en el Nao:

```bash
source ~/nao_ws/install/setup.bash
```

### Comunicación entre el ordenador y el Nao

Para que los nodos del ordenador y los del robot se vean, ambos tienen que estar en la misma red, con el mismo `ROS_DOMAIN_ID` y, preferiblemente, con la misma implementación de DDS. Si no se indica nada, ROS 2 Humble usa Fast DDS; si en uno de los dos lados se exporta `RMW_IMPLEMENTATION` (por ejemplo `rmw_cyclonedds_cpp`), haz lo mismo en el otro.

---

**Recomendación:** Es probable que haya que ajustar el volumen de los micrófonos del robot **Nao** con *amixer* (recomendable ponerlos al 90%):

```bash
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
* La interacción por voz a través de los servicios `stt_service` y `tts_service` (y las acciones `stt_action` y `tts_action`) de [simple_hri](https://github.com/rodperex/simple_hri).
* El servidor de posiciones del robot `nao_pos_action_server` y el de LEDs `led_action_server`.

## Simulación con Webots

La simulación se ejecuta en el ordenador. Webots simula el robot y su interfaz LoLa, y los mismos nodos que corren en el Nao se conectan a él.

1. Instala el simulador [Webots](https://cyberbotics.com/) (probado con R2025a):
   ```bash
   curl -L -o /tmp/webots.deb https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
   sudo apt install /tmp/webots.deb
   ```
2. Lanza Webots con el entorno del workspace cargado (el controlador del robot usa `rclpy` y `cv_bridge`) y selecciona alguno de los mundos proporcionados en [nao_webots](https://github.com/rodperex/nao_webots) (`src/nao_webots/worlds/`), por ejemplo:
   ```bash
   source ~/nao_ws/install/setup.bash
   webots ~/nao_ws/src/nao_webots/worlds/nao_indoors.wbt
   ```
3. En otro terminal, activa el robot (`simnao.launch.py` usa unos parámetros de marcha ajustados para el simulador):
   ```bash
   source ~/nao_ws/install/setup.bash
   ros2 launch nao_ros2 simnao.launch.py
   ```

Para que funcionen los servicios de voz en el ordenador hacen falta las mismas dependencias y credenciales que en el Nao (ver [Dependencias en el Nao](#dependencias-en-el-nao)).
