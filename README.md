# nao_ros2

![nao_ros2_logo](https://github.com/user-attachments/assets/7993fa06-eb04-4bc7-bc2d-a3a16c6948d7)

## 📌 Índice  
- [Introducción](#introducción)  
- [Instalación y Configuración](#instalación-y-configuración)  
  - [Requisitos Previos](#requisitos-previos)  
  - [Instalación de Ubuntu en el Nao](#instalación-de-ubuntu-en-nao)
  - [Configuración inicial de Ubuntu en el robot](#configuración-inicial-de-ubuntu-en-el-robot)
  - [Instalación de ROS 2](#instalación-de-ros-2)
  - [Open Access Nao (OAN)](#open-access-nao-oan)
  - [Primeros pasos y pruebas de los paquetes](#primeros-pasos-y-pruebas-de-los-paquetes)
  - [Configuración de Nav2](#configuración-de-nav2)  
- [Integración con API Web](#integración-con-api-web)  
- [Solución de Problemas y Buenas Prácticas](#solución-de-problemas-y-buenas-prácticas)  
- [Contribuciones](#contribuciones)  
- [Licencia](#licencia)  

---

<div id='introducción' />
  
## Introducción

El objetivo de este repositorio es documentar de manera clara y unificada el proceso de instalación y configuración de **Ubuntu 22.04** y **ROS 2** en el robot **Nao**. Actualmente, la información sobre este procedimiento está altamente distribuida y mal documentada, lo que dificulta su implementación. **nao_ros2** busca solucionar este problema proporcionando una guía completa y estructurada.

Este proyecto se centra en integrar **Nao** con **ROS 2**, permitiendo la navegación autónoma en un entorno mapeado y controlado a través de una API web externa. Para lograr esto, se ha adoptado [**Open Access Nao (OAN)**](https://github.com/antbono/OAN), un framework de código abierto desarrollado para habilitar el control y la locomoción del Nao en **ROS 2**. OAN proporciona una serie de paquetes modulares que permiten el acceso a los sensores y actuadores del robot, la reproducción de gestos, el control de LEDs, la locomoción mediante **walk**, y otras funciones esenciales. Su integración en este repositorio garantiza una plataforma robusta y flexible para la investigación y desarrollo con el Nao en entornos basados en ROS 2.

A lo largo de este repositorio, se detallarán los pasos necesarios para:  
- Instalar y configurar **Ubuntu 22.04** en el robot Nao.  
- Instalar **ROS 2** y los paquetes esenciales para el sistema.
- Integrar **OAN** para habilitar el control del robot en **ROS 2**.  
- Configurar **Nav2** para la navegación autónoma del robot.  
- Establecer comunicación con una API web para recibir comandos de destino y ejecutar movimientos precisos en el mapa.  

Este repositorio está dirigido a desarrolladores, investigadores y entusiastas de la robótica que deseen modernizar y optimizar las capacidades del robot **Nao** utilizando tecnologías de código abierto.

---

## Vídeos de ejemplo

Puedes encontrar los **vídeos de ejemplo** del proyecto en el siguiente canal de YouTube: [Robot Nao en ROS2](https://www.youtube.com/@RobotNaoenROS2)

 
---

<div id='instalación-y-configuración' />
  
## 🔧 Instalación y Configuración  

<div id='requisitos-previos' />

### 📌 Requisitos Previos  
Antes de comenzar, asegúrate de contar con:  
- Un robot **Nao V6** compatible.  
- Un ordenador con **Ubuntu 22.04** (Todavía no está probado en Ubuntu 24.04).  
- Conexión a **Internet**, un router con **tabla de IP accesible** y un **cable Ethernet**.
- **Usb drive** de al menos **4GB**.
- Conocimientos básicos de **Linux, ROS 2 y navegación robótica**.  

<div id='instalación-de-ubuntu-en-nao' />
  
### 🖥 Instalación de Ubuntu en el Nao  

Para instalar Ubuntu en el robot Nao V6, se debe seguir un proceso que involucra la **creación de una imagen de Ubuntu 22.04 a partir de una imagen oficial de NaoQi** (el sistema operativo predeterminado del Nao) y su posterior flasheo en el robot. Es **imprescindible por ahora utilizar la versión 2.8.5.10 de NaoQi**, ya que otras versiones pueden causar problemas de compatibilidad.

---

## **📈 Pasos para la creación de la imagen de Ubuntu**  

1. **Clonar el repositorio NaoImage**  
   ```bash
   git clone https://github.com/NaoDevils/NaoImage.git
   cd NaoImage
   ```

2. **Obtener la imagen correcta de NaoQi (2.8.5.10)**  
   - Obtener la versión 2.8.5.10 del software oficial de Aldebaran.

3. **Generar la imagen de Ubuntu**  
   - Ejecutar los siguientes comandos dentro del repositorio clonado para generar la imagen del sistema:  
   ```bash
   # Generar imagen del sistema basada en Ubuntu (requiere permisos root)
   sudo ./generate_image.sh {imagen_de_ubuntu.opn} image.ext3 ubuntu

   # Convertir la imagen generada a formato .opn
   sudo ./generate_opn.sh image.ext3 image.opn
   ```

---

## **📈 Flasheo de la imagen en el robot Nao**  

4. **Descargar e instalar el flasher oficial de NaoQi**  
   - Se encuentra disponible en la documentación de **SoftBank** o [aquí](https://support.unitedrobotics.group/en/support/solutions/articles/80001018812-nao-6-downloads)
   - Descomprimir y lanzar con permisos root:
    ```bash
    sudo ./flasher
    ```

5. **Preparar un USB con el firmware original de NaoQi**  
   - Formatear un USB de al menos **4GB** en formato **FAT32**.  
   - Flashear la **imagen oficial de NaoQi 2.8.5.10** (e.g. `nao-2.8.5.10.opn`), **!No la generada todavía!**, en el USB usando la herramienta **NaoFlasher** con Factory Reset activado.
   - Este paso es esencial ya que el firmware de los motores es distinto en la última versión de NaoQi (que suele venir instalada de fábrica en el Nao), y si se salta este paso los motores no funcionarán correctamente cuando se haya instalado Ubuntu.

6. **Realizar el primer flasheo con la imagen original de NaoQi**  
   - Apagar el Nao.  
   - Insertar el **USB con el firmware original** en el robot.  
   - Mantener presionado el botón del pecho durante **6 segundos o más** hasta que la luz del pecho se ponga azul.  
   - Esperar unos minutos, el botón cambia de color a blanco y se inicia el sistema. Confirmar que NaoQi está instalado fijándose en su comportamiento (habla, se mueve, pide conectarse a la red, cambia de colores a verde a veces, reacciona a toques en el pecho, etc).
   - **Opcional:** Se puede confirmar que la versión correcta de Naoqi (2.8.5.10) ha sido instalada a través de ssh (consultar [configuración de aldebaran](http://doc.aldebaran.com/2-8/family/nao_user_guide/introduction_nao.html)).

7. **Crear un USB con la imagen de Ubuntu**  
   - Formatear el USB de **4GB o más** en **FAT32**.  
   - Ahora sí, flashear la imagen de **Ubuntu 22.04 generada** (`image.opn`) en el USB utilizando el flasher, con Factory Reset activado. 

8. **Flashear la imagen de Ubuntu en el Nao**  
   - Apagar el robot nuevamente.  
   - Insertar el **USB con la imagen de Ubuntu**.  
   - Mantener presionado el botón del pecho durante **6 segundos o más** hasta que la luz del pecho se ponga azul. 
   - Esperar unos minutos, el botón cambia de color a blanco y se inicia el sistema (al principio parpadean algunos leds y luego se mantienen fijos, los ojos y pecho en blanco y las orejas en azul). Confirmar que ubuntu está instalado correctamente fijándose en su comportamiento (ya no habla, no se mueve, no reacciona a toques en el pecho).

---

## **✅ Verificación de instalación exitosa**  

👌 **NaoQi instalado correctamente (antes de Ubuntu):**  
- El robot **habla, se mueve** y reacciona al tacto.  
- Cambia de colores **(verde, azul, blanco, etc.)**.  
- Solicita conexión a la red.  

👌 **Ubuntu instalado correctamente:**  
- El robot **no habla ni se mueve** automáticamente.  
- Los LEDs de los ojos y el pecho se quedan **en blanco**.  
- Las orejas del robot quedan **en azul fijo**.  

En este punto, Ubuntu ya está instalado y el robot está listo para la configuración inicial y la instalación de **ROS 2**. 🚀

<div id='configuración-inicial-de-ubuntu-en-el-robot' />

### ⚙️ Configuración inicial de Ubuntu en el robot  

Una vez instalado Ubuntu en el robot Nao, es necesario realizar una configuración inicial para habilitar la conexión a la red, ajustar la configuración regional y preparar el sistema para su uso con ROS 2.

---

## **🌐 1. Conectar el robot Nao por Ethernet**  

1. Conectar el Nao a un router mediante un cable **Ethernet**.
2. Asegurarse de estar conectado en tu ordenador al wifi del router.
3. Acceder a la configuración del router a través de su dirección IP en un navegador web (por ejemplo, `192.168.1.1`).  
4. Identificar la dirección IP asignada al Nao en la tabla de dispositivos conectados.  
5. Conectarse por **SSH** al robot usando su dirección IP (ejemplo genérico):  
   ```bash
   # La contraseña es nao
   ssh nao@192.168.1.100
   ```  
6. **Opcional:** Habilitar el color en el terminal descomentando la siguiente línea en `.bashrc`:  
   ```bash
   sudo nano ~/.bashrc
   ```  
   Buscar y descomentar:  
   ```bash
   #force_color_prompt=yes
   ```

---

## **🗣️ 2. Configurar locales y solucionar errores de idioma**  
Si aparece un error sobre `LC_ALL`, las configuraciones regionales deben ajustarse:

1. Editar el archivo de configuración de localización:  
   ```bash
   sudo nano /etc/default/locale
   ```
2. Asegurarse de que contenga las siguientes líneas o ajustarlas si es necesario:  
   ```bash
   LANG=en_US.UTF-8
   LANGUAGE=en_US:en
   LC_ALL=en_US.UTF-8
   ```  
3. Regenerar las configuraciones regionales:  
   ```bash
   sudo locale-gen en_US.UTF-8
   sudo dpkg-reconfigure locales
   ```  
4. Reiniciar el robot para aplicar los cambios:  
   ```bash
   sudo reboot
   ```

---

## **🛜 3. Configurar conexión WiFi**  
Una vez conectado por **SSH**, se debe configurar `netplan` para que el robot se conecte automáticamente a una red WiFi:

1. Abrir el archivo de configuración de `netplan`:  
   ```bash
   sudo nano /etc/netplan/default.yaml
   ```
2. Modificar el archivo con la configuración de Ethernet y WiFi (ajustando los valores según la red utilizada):  
   ```yaml
   network:
     version: 2
     renderer: networkd
     ethernets:
       eth0:
         optional: true
         dhcp4: true
         dhcp6: false
         addresses:
           - 192.168.1.110/24  # Dirección para Ethernet (Ajustar al gusto)
         routes:
           - to: default
             via: 192.168.1.1
     wifis:
       wlan0:
         optional: true
         dhcp4: true
         dhcp6: false
         access-points:
           "Mi_Red_WiFi_SSID":
             password: "MiContraseñaSegura"
         addresses:
           - 192.168.1.120/24  # Dirección para Wi-Fi (Ajustar al gusto)
   ```
3. Aplicar la nueva configuración de red (no preocuparse por el warning):  
   ```bash
   sudo netplan apply
   ```
4. Verificar que el Nao ya puede conectarse por WiFi utilizando SSH con la nueva dirección IP (asegurase en las tablas del router de la ip que se le ha asignado por wifi):
   ```bash
   ssh nao@192.168.1.120
   ```
5. Reiniciar el robot y comprobar que se conecta automáticamente al wifi.



<div id='instalación-de-ros-2' />

### 🤖 Instalación de ROS 2 en el Nao y un PC 

Para que el robot Nao pueda operar correctamente con ROS 2, es mejor instalar la distribución **ROS 2 Rolling Ridley**, que es la versión de desarrollo continuo de ROS 2. A diferencia de las versiones estables como **Humble**, Rolling es una distribución en constante actualización, lo que permite acceder a las últimas mejoras, parches y compatibilidad con paquetes en desarrollo. Finalmente, hemos optado por usar **Rolling** porque muchos paquetes necesarios para el Nao, como **nao_lola** y **walk**, tienen un desarrollo más avanzado en esta versión.

ROS2 Rolling deberá ser instalado tanto en un pc con **Ubuntu 22.04** (o 24.04, pero todavía no está probado) como en el Nao. La diferencia clave es que la instalación en el Nao será una instalación base, ya que no tiene interfaz gráfica.

---

## **🛠️ Pasos para instalar ROS 2 Rolling en el PC y Nao**  

### **1. Configurar locales, repositorios y claves**  
Antes de instalar ROS 2, se deben configurar los locales, repositorios correctos y agregar las claves necesarias:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Ahora, agregar la clave GPG oficial de ROS 2:
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Finalmente, agregar el repositorio de ROS 2 a la lista de sources:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

### **2. Instalar ROS 2 Rolling**  
Actualizar el sistema antes de instalar ROS 2:
```bash
sudo apt update && sudo apt upgrade -y
```

Instalar herramientas de desarrollo de ROS:

```bash
sudo apt install ros-dev-tools
```

Actualiza los repositorios de apt después de configurarlos:

```bash
sudo apt update
```

ROS 2 se basa en sistemas Ubuntu actualizados. Se recomienda mantener el sistema al día antes de instalar paquetes:

```bash
sudo apt upgrade
```

Instalar ROS 2 Rolling en el **Nao** (versión base, sin entorno gráfico):
```bash
sudo apt install ros-rolling-ros-base
```
Este paquete incluye:
- Herramientas básicas de **ROS 2**
- Librerías de comunicación
- Soporte para **nodos, servicios y mensajes**

Instalar ROS 2 Rolling en el **PC** (version de escritorio, con entorno gráfico)
```bash
sudo apt install ros-rolling-desktop
```
Este paquete incluye además:
- Herramientas de visualización como RViz
- Simulador Gazebo (dependiendo de la versión)
- Soporte para rqt y sus complementos
- Herramientas adicionales para desarrollo y depuración

---

### **3. Configurar el entorno**  
Se debe configurar el entorno para que ROS 2 se cargue en cada sesión:
```bash
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### **4. Verificar la instalación de ROS 2**  
Comprobar que ROS 2 está correctamente instalado:

- Mostrar la ayuda del cliente ROS 2:
```bash
ros2 --help
```
- Listar los paquetes instalados:
```bash
ros2 pkg list
```

Si ambos comandos muestran información correctamente, la instalación de ROS 2 Rolling ha sido exitosa. El Nao y el PC están listos para continuar con la configuración de paquetes adicionales como **nao_lola** y **walk**. 🚀

<div id='open-access-nao-oan' />

### 🦾 Open Access Nao (OAN)

#### Introducción

[**Open Access Nao (OAN)**](https://github.com/antbono/OAN) es un conjunto de paquetes desarrollados para habilitar la integración del robot **Nao V6** en entornos **ROS 2**, proporcionando herramientas avanzadas para su control, operación y experimentación en diferentes áreas de la robótica. Este ecosistema permite interactuar con el Nao de manera modular y eficiente, ofreciendo un entorno accesible para su movimiento, locomoción e interacción en diversos escenarios de investigación y desarrollo.

La motivación detrás de OAN radica en la necesidad de contar con herramientas más abiertas y accesibles para el desarrollo en el Nao V6, facilitando el trabajo de la comunidad científica y de ingeniería. Este framework ha sido posible gracias al trabajo de expertos como [**Antonio Bono (antbono)**](https://github.com/antbono) y [**Kenji Brameld (ijnek)**](https://github.com/ijnek), quienes han desarrollado múltiples paquetes fundamentales para habilitar la ejecución de **ROS 2** en el Nao y aprovechar sus capacidades en términos de percepción y actuación.

#### Estructura de OAN

El ecosistema de OAN está compuesto por múltiples paquetes interconectados, cada uno con una función específica dentro del sistema. A continuación, se detallan los principales paquetes y sus funciones:

1. **HRI (Human-Robot Interaction)**
   - Funciona como un middleware central que facilita la comunicación entre diferentes módulos del sistema.
   - Optimiza la interacción entre los paquetes de percepción, locomoción y control del Nao, garantizando una integración fluida y coordinada.

2. **NAO_LoLA (Low-Level Access)**
   - Implementa la interfaz **LoLA (Low-Level Access)** del Nao V6, permitiendo el acceso directo a sensores y actuadores del robot.
   - Proporciona datos en tiempo real sobre el estado de los motores, sensores de fuerza, IMU, cámaras y otros componentes del hardware del Nao.
   - Permite controlar los actuadores como los motores de las articulaciones y los leds del Nao.

3. **NAO_POS** 
   - Facilita la ejecución de gestos y movimientos predefinidos en el robot Nao.
   - Permite realizar transiciones suaves entre distintas posturas, mejorando la expresividad del robot.

4. **NAO_LED** (Desarrollado por Antonio Bono)
   - Proporciona un control avanzado del sistema de iluminación del Nao.
   - Permite utilizar luces LED para representar diferentes estados internos del robot mediante colores y patrones personalizados.

5. **WALK** (Integrado dentro de OAN, desarrollado por **Kenji Brameld**)
   - Sistema de locomoción desarrollado en **ROS 2** diseñado específicamente para Nao y otros robots bípedos.
   - Proporciona un controlador de marcha que optimiza la estabilidad del robot al desplazarse.
   - Se integra con **nao_ik** y **nao_phase_provider**.

6. **NAO_IK (Inverse Kinematics, desarrollado por Kenji Brameld)**
   - Implementa **cinemática inversa** en el Nao. Su objetivo es calcular las configuraciones articulares necesarias para caminar.

7. **NAO_PHASE_PROVIDER (Desarrollado por Kenji Brameld)**
   - Se encarga de detectar la presión con el suelo de los pies del Nao gracias a los cuatro sensores de resistencia sensible a la fuerza montados en cada pie, para el posterior cálculo del movimiento bípedo.

8. **AUDIO_COMMON y USB_CAM** (Soporte adicional)
   - **AUDIO_COMMON** gestiona la grabación y reproducción de sonido en el Nao, permitiendo implementar sistemas de reconocimiento y respuesta auditiva.
   - **USB_CAM** proporciona compatibilidad con las cámaras del nao, facilitando la integración de visión computacional en ROS 2.
   - Ambos paquetes amplían la capacidad de percepción del robot, permitiendo el desarrollo de aplicaciones más completas.
  
9. **SYNC**
   - Script desarrollado por [**Kenji Brameld (ijnek)**](https://github.com/ijnek/sync) que permite sincronizar un robot, como Nao, con un **workspace precompilado** de ROS 2 en otro pc. Su propósito principal es resolver el problema de la **compilación lenta en el Nao**, ya que el robot tiene **recursos computacionales limitados** y tarda mucho en compilar paquetes complejos.

#### Créditos y Reconocimiento

El desarrollo de **OAN** ha sido posible gracias al esfuerzo de múltiples colaboradores, con especial reconocimiento a **Antonio Bono (antbono)** y **Kenji Brameld (ijnek)**. 

- **Antonio Bono (antbono)** ha liderado la integración de **Nao con ROS 2**, estableciendo una base sólida para el Human Robot Interaction (HRI) en el Nao.
- **Kenji Brameld (ijnek)** ha realizado contribuciones esenciales para la locomoción y la sincronización de workspaces.

El trabajo conjunto de estos desarrolladores ha permitido que **OAN** se convierta en una plataforma confiable y robusta para la investigación y desarrollo del Nao en **ROS 2**. Gracias a estas contribuciones, **OAN** es actualmente una de las plataformas más completas para trabajar con el **Nao en ROS 2**.

### **Instalación de dependencias en el pc y en el nao**

> :warning: **Aviso**: Si se quiere utilizar webots para simular el nao en un pc, es necesario instalar las dependencias necesarias en el pc además de en el nao.

Es necesario seguir los pasos explicados en el repositorio [**OAN**](https://github.com/antbono/OAN) de Antonio Bono **¡¡CUIDADO!!**. Por ahora es necesario saltarse los pasos de instalación del readme de OAN **(2.1 y 2.3)**, ya que los repositorios necesarios se instalarán después. Esto se debe a que algunos repositorios están modificados y es necesario clonar mis fork en vez del original. Ignorar el **aviso** de audio_common en el paso **2.2** ya que en rolling no existe este problema. De la sección **3** es **solo** necesario realizar la **3.3**  ya que el resto ya han sido realizadas y explicadas en este readme (si se quiere usar el simulador **webots**, también es necesario hacer este paso **en tu pc**).

#### **Instalación de repositorios de OAN**

Para utilizar **OAN** en el robot Nao, es necesario clonar e instalar los paquetes requeridos en un pc. Se utilizarán los paquetes de **OAN**, salvo los forks personalizados de algunos módulos específicos.

#### **💻 Clonación de paquetes**

```bash
mkdir -p nao_ws/src
vcs import src < thirdparty.repos --recursive

```

Para comprobar las ramas activas de todos los repositorios dentro del directorio src, se puede utilizar el siguiente comando optimizado:

```bash
for repo in */.git; do echo ""; echo "📂 Repo: ${repo%/.git}"; git -C "${repo%/.git}" status; echo ""; echo "Remote:"; git -C "${repo%/.git}" remote -v; echo ""; echo "-------------------------------------------------------------------------"; done
```

#### **🛠 Instalación de dependencias**

Una vez clonados los paquetes, se deben instalar sus dependencias:

```bash
cd ~/nao_ws
rosdep update
rosdep install --from-paths src -r -y
```

Además, instalar estas librerías (tanto en el PC como en el Nao):
```bash
sudo apt-get install libmsgpack-dev
sudo apt-get install libignition-transport11-dev
pip install webrtcvad
pip install sounddevice
```

---

#### **🚀 Compilación e instalación**

Compilar los paquetes usando `colcon` para generar los binarios necesarios :

```bash
colcon build
```

**IMPORTANTE**: No compilar utilizando la opción `--symlink-install` para poder utilizar posteriormente `sync` para llevar el directorio /install al robot Nao

Seguir los pasos del repositorio de Kenji Brameld con el objetivo de utilizar [sync](https://github.com/ijnek/sync) para cargar el workspace compilado en el nao.

Finalmente, añadir el source al `bashrc` (tanto en el pc como en el nao):

```bash
echo "source ~/nao_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
---

**Recomendación en el Nao:** Es probable que haya que ajustar el volumen de los micrófonos del Nao con amixer (recomendable ponerlos al 90%):

```bash
sudo apt update && sudo apt install alsa-utils
amixer set 'Numeric Left mics' 90% cap
amixer set 'Numeric Right mics' 90% cap
amixer set 'Analog Front mics' 90% cap
amixer set 'Analog Rear mics' 90% cap
``` 

#### **🔄 ¿Por qué usar los forks personalizados?**

A continuación se explica por qué se han utilizado forks personalizados en lugar de los repositorios originales:

- **nao_hri (fork de de antbono/hni)**:
  - Paquete adaptado al español para mejorar la interacción del robot de rehabilitación de personas mayores, ubicado en el Laboratorio de Robótica y Sistemas Ubícuos de la Escuela de Ingeniería de Fuenlabrada (URJC). Se ha optimizado el uso de STT con OpenAI Whisper, ya que ofrece una mejor detección del español y un VAD más preciso y rápido para la generación de audios. Además, se han implementado mejoras en modelos de GPT más modernos, rápidos, eficientes y económicos.
  - Mode switcher añadido para combinar el walk y el speech. 
- **nao\_lola (fork de ijnek)**:
  - `nao_command_msgs` renombrado a `nao_lola_command_msgs` para alinearse correctamente con el tipo de mensaje utilizado en todos los paquetes en rolling y en mis fork.
- **nao\_pos (fork de antbono)**:
  - Se han realizado mejoras en la sincronización del inicio del robot, incorporación de nuevos gestos, compatibilidad y suscripción al estado de walk para optimizar su comportamiento.
- **walk (fork de ijnek)**:
  - Fallos corregidos en algunos include en el repositorio original
  - Añadida detección de caídas para detener el walk y levantarse automáticamente.
  - Lógica para coordinar el walk y el modo speech.


### Instalación y configuración del simulador Webots

Para simular el Nao en un pc es necesario instalar el simulador **Webots** y clonar mi fork del repositorio [**WebotsLoLaController**](https://github.com/andoniroldan/WebotsLoLaController.git) de [bembelbots](https://bembelbots.de/) para utilizar el mundo con el Nao. Este repositorio contiene dos mundos que permiten la conexión con nao_lola como si fuera el robot real.

Instalar Webots desde la [página oficial](https://cyberbotics.com/)

Clonar el fork del repositorio WebotsLoLaController (rama ros2_camera_publish):
```bash
git clone -b ros2_camera_publish https://github.com/andoniroldan/WebotsLoLaController.git
```

Abrir Webots desde un terminal (hacer source de ros previamente):
```bash
source /opt/ros/rolling/setup.bash
webots
```

Dentro de webots seleccionar File -> Open World... -> Seleccionar un mundo de la carpeta worlds del repositorio WebotsLoLaController (para usar ros2_camera_publish, WebotsLoLaController/worlds/nao_robocup.wbt)

Lanzar nao_lola en una terminal como si estuvieras en el robot real
```bash
ros2 run nao_lola_client nao_lola_client
```

Debería aparecer en verde 'LoLa client connected' en la consola de Webots.


### 🔬 Simulación con Webots y Publicación de Cámaras en ROS 2  

Se ha realizado una mejora en el simulador **Webots**, añadiendo una funcionalidad clave para mejorar la integración de visión artificial en ROS 2:  

- **Publicación de la imagen de la cámara simulada del Nao en el topic `/image_raw`**  
  - Permite utilizar herramientas como **YOLO** dentro del simulador, sin necesidad de emplear la cámara del ordenador.  
  - Facilita la experimentación y pruebas de visión artificial sin necesidad del robot físico.  

Para ello:  
- Se ha creado un **fork de WebotsLoLaController**, en la rama [`ros2_camera_publish`](https://github.com/andoniroldan/WebotsLoLaController/tree/ros2_camera_publish), que:  
  - Publica la imagen simulada en **ROS 2**.  
  - Añade un objeto con una cara dentro del mundo del simulador (para probar detección con YOLO).

- También se ha modificado **HRI**, en la rama [`simulation`](https://github.com/geriabot/nao_hri/tree/simulation), para:  
  - Evitar el uso de `usb_cam`, que podría interferir con la cámara simulada.  
  - Ajustar el tratamiento de la imagen simulada para hacerla compatible con **YOLO**.  

#### 🚀 **Cómo probar el seguimiento de caras en simulación**  
1. **Iniciar Webots** desde un terminal (para que haga `source` de ROS 2).  
2. Seleccionar el mundo **"nao_robocup"** en Webots.  
3. Lanzar los **dos experiment launchers de nao_hri** (como en la ejecución en el robot real).  

Esto permitirá que el Nao simulado detecte y siga caras dentro del entorno virtual (presionar shift y arrastrar para mover el objeto de la cara humana)

---

<div id='primeros-pasos-y-pruebas-de-los-paquetes' />

### 🤔 Primeros pasos y pruebas de los paquetes

# 🏃‍♂️ Uso del ModeSwitcher en Simulación

El **ModeSwitcher** es un nodo de ROS 2 diseñado para gestionar el inicio y la detención de la locomoción del Nao. Además de caminar, este sistema permite que el Nao hable e interactúe mediante gestos predefinidos, acercándose a un modelo más cognitivo y completo de interacción humano-robot (HRI). Con esta herramienta, el Nao no solo se desplaza y camina, sino que también responde y actúa en un entorno dinámico, mejorando la interacción con el usuario.

## **1️⃣ Configuración previa**

Antes de ejecutar el `mode_switcher`, es necesario definir la variable de entorno `NAO_WS_PATH`, que contiene la ruta de tu workspace de ROS 2:

```bash
export NAO_WS_PATH=/path_al_workspace  # Ajusta la ruta según corresponda
```

Para hacer que esta variable sea permanente, agrégala al final de tu archivo `~/.bashrc` y haz source:

```bash
echo "export NAO_WS_PATH=/path_al_workspace" >> ~/.bashrc
source ~/.bashrc
```

## **2️⃣ Lanzamiento del ModeSwitcher**

Para iniciar el nodo asegurate de estar en la rama simulation en el repositorio hni, compila y haz source. Abre webots en el mundo del nao y ejecuta el siguiente comando en una terminal:

```bash
ros2 run hni_py mode_switcher
```

Este comando iniciará el nodo y activará el teleoperador, permitiéndote controlar los modos y el movimiento del Nao.

## **3️⃣ Modos de operación**

- **Caminar:** Mantén presionadas las teclas del teleoperador que se abre automáticamente. El modo cambiará a **walk**, el robot se preparará y comenzará a andar.
- **Detenerse:** Mantén presionada la tecla `k` para detener el **walk** y volver al modo interactivo.
- **Modo interactivo:** Cuando el Nao está detenido, puedes interactuar con él a través de la terminal **x-term** que se abre automáticamente. Para hablar con el Nao, presiona `Enter` en esa terminal y háblale. Cada vez que quieras decirle algo, debes repetir este proceso. Es recomendable esperar a que termine de hablar para evitar solapar voces y ajustar el volumen del micrófono a un nivel bastante bajo para que el reconocimiento de voz funcione correctamente.
- **Gestos y animaciones:** El Nao puede realizar ciertos gestos predefinidos como **decir hola, bailar o hacer el gesto de grande**. Estos gestos se activan cuando el Nao los menciona en su respuesta, por lo que puedes pedirle que los realice hablándole.
- Este sistema cuenta con una función que permite al Nao levantarse automáticamente en caso de caída, utilizando los datos de sus sensores de aceleración para detectarlo. No obstante, aunque los gestos funcionan correctamente en el mundo real, en el simulador el proceso de levantarse puede presentar dificultades y no ejecutarse con la misma precisión.

> **Nota:** Es recomendable mantener las teclas presionadas en **teleop** para lograr un control más preciso del movimiento. Este sistema ha sido diseñado para garantizar compatibilidad con **Nav2** en futuras implementaciones.

## **4️⃣ Detención del ModeSwitcher**

Para detener el nodo, presiona `Ctrl + C` en la terminal donde lo ejecutaste y cierra todas las ventanas.

Con este sistema, el Nao no solo camina, sino que también mejora la interacción con el usuario, avanzando hacia un modelo más natural e intuitivo de interacción robot-humano.

# 🤖 Uso del ModeSwitcher en el Robot Real

El **ModeSwitcher** es un sistema distribuido que gestiona el inicio y la detención de la locomoción del Nao, esta vez en el mundo real. Además de caminar, este sistema permite que el Nao hable, siga rostros y realice gestos predefinidos, acercándose a un modelo más cognitivo y completo de interacción humano-robot (HRI). Con esta herramienta, el Nao no solo se desplaza de forma real en el entorno, sino que también responde e interactúa con los usuarios de manera segura y natural.

## **1️⃣ Configuración previa**

Antes de ejecutar el `mode_switcher_nao` o `mode_switcher_pc`, es necesario definir la variable de entorno `NAO_WS_PATH`, que contiene la ruta de tu workspace de ROS 2 (hazlo tanto en el pc como en el nao):

```bash
export NAO_WS_PATH=/path_al_workspace  # Ajusta la ruta según corresponda
```

Para hacer que esta variable sea permanente, agrégala al final de tu archivo `~/.bashrc` y haz source:

```bash
echo "export NAO_WS_PATH=/path_al_workspace" >> ~/.bashrc
source ~/.bashrc
```

## **2️⃣ Lanzamiento del ModeSwitcher**

Antes de iniciar los nodos, asegúrate de clonar este repositorio (`nao_ros2`), compilar el workspace y hacer `source` del entorno.

Luego, sincroniza la compilación al robot Nao usando el script `sync.sh`, que permite copiar tu workspace ya compilado al robot de forma rápida mediante SSH.

Ubícate dentro del workspace que deseas sincronizar y ejecuta:

```bash
./sync.sh nao <ip_nao>
```

Este comando actualizará automáticamente el entorno del robot con la última compilación realizada en tu PC.



Antes de lanzar el nodo en el Nao, asegúrate de hacer `source` del entorno compilado en el robot para que ROS 2 reconozca los ejecutables:

```bash
source ~/nao_ws/install/setup.bash
```



Para lanzar el ModeSwitcher, primero debes lanzar el launcher en el **robot Nao** con el siguiente comando (asegurate de estar en el directorio nao_ws):

```bash
ros2 launch nao_ros2 mode_switcher_nao_launch.py
```

Luego, en el **PC**, lanza el launcher correspondiente (asegurate de estar en el directorio nao_ws):

```bash
ros2 launch nao_ros2 mode_switcher_pc_launch.py
```

Esto activará el sistema completo de locomoción e interacción.

## **3️⃣ Modos de operación**

- **Caminar:** Mantén presionadas las teclas del teleoperador que se abre automáticamente en el pc. El modo cambiará a **walk**, el robot se preparará y comenzará a andar.
- **Detenerse:** Mantén presionada la tecla `k` para detener el **walk** y volver al modo interactivo.
- **Modo interactivo:** Cuando el Nao está detenido, puedes interactuar con él a través de la terminal **x-term** que se abre automáticamente en el pc. Para hablar con el Nao, presiona `Enter` en esa terminal y háblale. Cada vez que quieras decirle algo, debes repetir este proceso.
- **Gestos y animaciones:** El Nao puede realizar ciertos gestos predefinidos como **decir hola, bailar o hacer el gesto de grande**. Estos gestos se activan cuando el Nao los menciona en su respuesta, por lo que puedes pedirle que los realice hablándole.
- **Seguridad mejorada:** En el mundo real, el sistema de locomoción cuenta con mecanismos de seguridad adicionales. Si el robot **cae**, detecta la caída con sensores de aceleración y se levanta automáticamente. Además, si los **sensores de presión** de los pies no detectan contacto con el suelo, el robot detiene el movimiento de inmediato para evitar accidentes.

> **Nota:** Es recomendable mantener las teclas presionadas en **teleop** para lograr un control más preciso del movimiento. Este sistema ha sido diseñado para garantizar compatibilidad con **Nav2** en futuras implementaciones.

## **4️⃣ Detención del ModeSwitcher**

Para detener los nodos, presiona `Ctrl + C` tanto en el Nao como en el PC, y cierra todas las ventanas abiertas.

Con este sistema, el Nao no solo camina de forma real, sino que también mejora la interacción con el usuario, avanzando hacia un modelo más natural, seguro e intuitivo de interacción robot-humano.


---

<div id='configuración-de-nav2' />

## 🚀 Configuración y Lanzamiento de SLAM y Navegación Autónoma

Esta sección explica cómo lanzar el sistema de navegación autónoma en el Nao usando SLAM (Simultaneous Localization and Mapping) y cómo utilizar el mapa generado para navegación posterior. Se detalla el orden de los comandos y qué hace cada uno. **Recuerda que la navegación depende de la correcta configuración de tu archivo `nav2_params.yaml`**, que contiene los parámetros clave para el stack de navegación (Nav2), tanto para SLAM como para la navegación sobre un mapa ya guardado.

---

### 1. Lanzamiento de SLAM: Crear el mapa del entorno

**SLAM** permite que el Nao construya un mapa del entorno y se localice en tiempo real usando el LiDAR y la odometría.

#### **En el Nao**

Lanza el nodo principal para locomoción e interacción:
```bash
ros2 launch nao_ros2 mode_switcher_nao_launch.py 
```

#### **En el PC**

1. Lanza el nodo de control desde el PC:
   ```bash
   ros2 launch nao_ros2 mode_switcher_pc_launch.py 
   ```
2. Publica la transformación estática entre la cabeza y el láser:
   ```bash
   ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 3.14 0.0 0.0 Head laser
   ```
3. Lanza la odometría para fusionar sensores y calcular la posición del robot:
   ```bash
   ros2 launch nao_ros2 nao_odometry_launch.py
   ```
4. Lanza el driver del LiDAR S2:
   ```bash
   ros2 launch sllidar_ros2 sllidar_s2_launch.py
   ```
5. Abre RViz2 para visualizar el mapa, la posición del robot y los sensores:
   ```bash
   ros2 run rviz2 rviz2
   ```

#### **En Distrobox (Nav2 / SLAM Toolbox)**

6. Lanza Nav2 con SLAM activado.  
   Usa un mapa vacío y tu archivo `nav2_params.yaml` (ajusta la ruta según corresponda):
   ```bash
   ros2 launch nav2_bringup bringup_launch.py \
     slam:=True \
     map:=/home/andoni/empty_map.yaml \
     params_file:=/home/andoni/nav2_params.yaml
   ```

   - Esto levanta el stack de navegación y SLAM Toolbox, que irá generando el mapa en tiempo real.

7. (Opcional) Si necesitas más control sobre SLAM Toolbox, puedes lanzarlo manualmente y remapear la odometría:
   ```bash
   ros2 launch slam_toolbox online_async_launch.py \
     params_file:=/home/andoni/nav2_params.yaml /odom:=/odometry/filtered
   ```

---

### 2. Navegación autónoma sobre mapa ya guardado

Una vez generado y guardado el mapa (`mi_mapa.yaml`), puedes usar el sistema de navegación para moverte a objetivos específicos en el entorno.

#### **En el Nao**

```bash
ros2 launch nao_ros2 mode_switcher_nao_launch.py 
```

#### **En el PC**

1. Relé de comandos de velocidad para conectar Nav2 con el controlador del Nao:
   ```bash
   ros2 run topic_tools relay /cmd_vel /target
   ```
2. Publica la transformación estática entre la cabeza y el láser:
   ```bash
   ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 3.14 0.0 0.0 Head laser
   ```
3. Lanza la odometría:
   ```bash
   ros2 launch nao_ros2 nao_odometry_launch.py
   ```
4. Abre RViz2 para visualizar navegación:
   ```bash
   ros2 run rviz2 rviz2
   ```
5. Lanza el LiDAR:
   ```bash
   ros2 launch sllidar_ros2 sllidar_s2_launch.py
   ```
6. Lanza el modo de control desde el PC:
   ```bash
   ros2 launch nao_ros2 mode_switcher_pc_launch.py 
   ```

#### **En el PC (distrobox, ROS 2 Humble)**

7. Lanza Nav2 con el mapa previamente guardado y tu archivo de parámetros:
   ```bash
   ros2 launch nav2_bringup bringup_launch.py \
     params_file:=/home/andoni/nav2_params.yaml \
     map:=/home/andoni/mi_mapa.yaml
   ```

---

### 📝 ¿Qué contiene `nav2_params.yaml` y por qué es importante?

El archivo `nav2_params.yaml` define los parámetros críticos del sistema de navegación: configuración de planners, controladores, sensores (odometría, LiDAR), comportamiento de recuperación, límites de velocidad, resolución del mapa y mucho más. **Asegúrate de adaptar este archivo a tu entorno y hardware, ya que una mala configuración puede provocar fallos en la navegación.**  
Puedes consultar ejemplos y recomendaciones en la documentación oficial de Nav2.

---

### 🛠️ Resumen: ¿Qué hace cada comando?

- **mode_switcher_nao_launch.py / mode_switcher_pc_launch.py**: Lanzan los nodos de locomoción y HRI tanto en el Nao como en el PC.
- **static_transform_publisher**: Publica la transformación fija entre el marco de la cabeza del Nao y el láser.
- **nao_odometry_launch.py**: Publica la odometría fusionada del Nao.
- **sllidar_s2_launch.py**: Lanza el driver del LiDAR para obtener los datos de escaneo del entorno.
- **rviz2**: Herramienta de visualización para ver mapa, sensores, posición del robot, objetivos, etc.
- **nav2_bringup bringup_launch.py**: Inicia el stack de navegación Nav2, usando los parámetros de `nav2_params.yaml` y el mapa que corresponda.
- **slam_toolbox online_async_launch.py**: Permite ejecutar SLAM para la creación del mapa en tiempo real.
- **topic_tools relay**: Redirige los comandos de velocidad generados por Nav2 hacia el controlador real del Nao.

---

> **Nota:**  
> 1. Primero utiliza SLAM para generar y guardar el mapa.  
> 2. Luego lanza Nav2 sobre el mapa generado para navegación autónoma.  
> 3. ¡No olvides ajustar y revisar el contenido de `nav2_params.yaml` para tu caso concreto!


---

<div id='integración-con-api-web' />

## 🌐 Integración con API Web  
Explicación de cómo el robot se comunica con la API para recibir comandos de navegación.  

---

<div id='solución-de-problemas-y-buenas-prácticas' />

## 🛠 Solución de Problemas y Buenas Prácticas  
Lista de errores comunes, cómo solucionarlos y recomendaciones para optimizar el sistema.  

---

<div id='contribuciones' />

## 🤝 Contribuciones  
Si quieres mejorar este proyecto, revisa nuestra [Guía de Contribución](#).  

---

<div id='licencia' />

## 📜 Licencia  
