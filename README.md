# nao_ros2

![nao_ros2_logo](https://github.com/user-attachments/assets/7993fa06-eb04-4bc7-bc2d-a3a16c6948d7)

## 📌 Índice  
- [Introducción](#introducción)  
- [Instalación y Configuración](#instalación-y-configuración)  
  - [Requisitos Previos](#requisitos-previos)  
  - [Instalación de Ubuntu en el Nao](#instalación-de-ubuntu-en-nao)
  - [Configuración inicial de Ubuntu en el robot](#configuración-inicial-de-ubuntu-en-el-robot)
  - [Instalación de ROS 2](#instalación-de-ros-2)
  - [Open Access NAO (OAN)](#open-access-nao-oan)
  - [Configuración de Nav2](#configuración-de-nav2)  
- [Integración con API Web](#integración-con-api-web)  
- [Solución de Problemas y Buenas Prácticas](#solución-de-problemas-y-buenas-prácticas)  
- [Contribuciones](#contribuciones)  
- [Licencia](#licencia)  

---

<div id='introducción' />
  
## Introducción

El objetivo de este repositorio es documentar de manera clara y unificada el proceso de instalación y configuración de **Ubuntu 22.04** y **ROS 2** en el robot **Nao**. Actualmente, la información sobre este procedimiento está altamente distribuida y mal documentada, lo que dificulta su implementación. **nao_ros2** busca solucionar este problema proporcionando una guía completa y estructurada.

Este proyecto se centra en integrar **Nao** con **ROS 2**, permitiendo la navegación autónoma en un entorno mapeado y controlado a través de una API web externa. Para lograr esto, se ha adoptado [**Open Access NAO (OAN)**](https://github.com/antbono/OAN), un framework de código abierto desarrollado para habilitar el control y la locomoción del NAO en **ROS 2**. OAN proporciona una serie de paquetes modulares que permiten el acceso a los sensores y actuadores del robot, la reproducción de gestos, el control de LEDs, la locomoción mediante **walk**, y otras funciones esenciales. Su integración en este repositorio garantiza una plataforma robusta y flexible para la investigación y desarrollo con el NAO en entornos basados en ROS 2.

A lo largo de este repositorio, se detallarán los pasos necesarios para:  
- Instalar y configurar **Ubuntu 22.04** en el robot Nao.  
- Instalar **ROS 2** y los paquetes esenciales para el sistema.
- Integrar **OAN** para habilitar el control del robot en **ROS 2**.  
- Configurar **Nav2** para la navegación autónoma del robot.  
- Establecer comunicación con una API web para recibir comandos de destino y ejecutar movimientos precisos en el mapa.  

Este repositorio está dirigido a desarrolladores, investigadores y entusiastas de la robótica que deseen modernizar y optimizar las capacidades del robot **Nao** utilizando tecnologías de código abierto.

 
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
   ./generate_opn.sh image.ext3 image.opn
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

Una vez instalado Ubuntu en el robot NAO, es necesario realizar una configuración inicial para habilitar la conexión a la red, ajustar la configuración regional y preparar el sistema para su uso con ROS 2.

---

## **🌐 1. Conectar el robot NAO por Ethernet**  

1. Conectar el NAO a un router mediante un cable **Ethernet**.
2. Asegurarse de estar conectado en tu ordenador al wifi del router.
3. Acceder a la configuración del router a través de su dirección IP en un navegador web (por ejemplo, `192.168.1.1`).  
4. Identificar la dirección IP asignada al NAO en la tabla de dispositivos conectados.  
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
3. Aplicar la nueva configuración de red:  
   ```bash
   sudo netplan apply
   ```
4. Verificar que el NAO ya puede conectarse por WiFi utilizando SSH con la nueva dirección IP (asegurase en las tablas del router de la ip que se le ha asignado por wifi):
   ```bash
   ssh nao@192.168.1.120
   ```
5. Reiniciar el robot y comprobar que se conecta automáticamente al wifi.



<div id='instalación-de-ros-2' />

### 🤖 Instalación de ROS 2 en el NAO  

Para que el robot NAO pueda operar correctamente con ROS 2, es mejor instalar la distribución **ROS 2 Rolling Ridley**, que es la versión de desarrollo continuo de ROS 2. A diferencia de las versiones estables como **Humble**, Rolling es una distribución en constante actualización, lo que permite acceder a las últimas mejoras, parches y compatibilidad con paquetes en desarrollo. Finalmente, hemos optado por usar **Rolling** porque muchos paquetes necesarios para el NAO, como **nao_lola** y **walk**, tienen un desarrollo más avanzado en esta versión.

ROS2 Rolling deberá ser instalado tanto en un pc con **Ubuntu 22.04** (o 24.04, pero todavía no está probado) como en el Nao. La diferencia clave es que la instalación en el Nao será una instalación base, ya que no tiene interfaz gráfica.

---

## **🛠️ Pasos para instalar ROS 2 Rolling en el PC y NAO**  

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

**Opcional:** Instalar herramientas de desarrollo de ROS:

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

Instalar ROS 2 Rolling en el **NAO** (versión base, sin entorno gráfico):
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
Este paquete inclute además:
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

Si ambos comandos muestran información correctamente, la instalación de ROS 2 Rolling ha sido exitosa. El NAO y el PC están listos para continuar con la configuración de paquetes adicionales como **nao_lola** y **walk**. 🚀

<div id='open-access-nao-oan' />

### 🦾 Open Access NAO (OAN)

#### Introducción

[**Open Access NAO (OAN)**](https://github.com/antbono/OAN) es un conjunto de paquetes desarrollados para habilitar la integración del robot **NAO V6** en entornos **ROS 2**, proporcionando herramientas avanzadas para su control, operación y experimentación en diferentes áreas de la robótica. Este ecosistema permite interactuar con el NAO de manera modular y eficiente, ofreciendo un entorno accesible para su movimiento, locomoción e interacción en diversos escenarios de investigación y desarrollo.

La motivación detrás de OAN radica en la necesidad de contar con herramientas más abiertas y accesibles para el desarrollo en el NAO V6, facilitando el trabajo de la comunidad científica y de ingeniería. Este framework ha sido posible gracias al trabajo de expertos como [**Antonio Bono (antbono)**](https://github.com/antbono) y [**Kenji Brameld (ijnek)**](https://github.com/ijnek), quienes han desarrollado múltiples paquetes fundamentales para habilitar la ejecución de **ROS 2** en el NAO y aprovechar sus capacidades en términos de percepción y actuación.

#### Estructura de OAN

El ecosistema de OAN está compuesto por múltiples paquetes interconectados, cada uno con una función específica dentro del sistema. A continuación, se detallan los principales paquetes y sus funciones:

1. **HNI (Human-Nao Interaction)**
   - Funciona como un middleware central que facilita la comunicación entre diferentes módulos del sistema.
   - Optimiza la interacción entre los paquetes de percepción, locomoción y control del NAO, garantizando una integración fluida y coordinada.

2. **NAO_LoLA (Low-Level Access)**
   - Implementa la interfaz **LoLA (Low-Level Access)** del NAO V6, permitiendo el acceso directo a sensores y actuadores del robot.
   - Proporciona datos en tiempo real sobre el estado de los motores, sensores de fuerza, IMU, cámaras y otros componentes del hardware del NAO.
   - Permite controlar los actuadores como los motores de las articulaciones y los leds del Nao.

3. **NAO_POS**
   - Facilita la ejecución de gestos y movimientos predefinidos en el robot NAO.
   - Permite realizar transiciones suaves entre distintas posturas, mejorando la expresividad del robot.

4. **NAO_LED**
   - Proporciona un control avanzado del sistema de iluminación del NAO.
   - Permite utilizar luces LED para representar diferentes estados internos del robot mediante colores y patrones personalizados.

5. **WALK** (Integrado dentro de OAN, desarrollado por **Kenji Brameld**)
   - Sistema de locomoción desarrollado en **ROS 2** diseñado específicamente para NAO y otros robots bípedos.
   - Proporciona un controlador de marcha que optimiza la estabilidad del robot al desplazarse.
   - Se integra con **nao_ik** y **nao_phase_provider**.

6. **NAO_IK (Inverse Kinematics, desarrollado por Kenji Brameld)**
   - Implementa **cinemática inversa** en el NAO. Su objetivo es calcular las configuraciones articulares necesarias para caminar.

7. **NAO_PHASE_PROVIDER**
   - Se encarga de detectar la presión con el suelo de los pies del NAO gracias a los cuatro sensores de resistencia sensible a la fuerza montados en cada pie, para el posterior cálculo del movimiento bípedo.

8. **AUDIO_COMMON y USB_CAM** (Soporte adicional)
   - **AUDIO_COMMON** gestiona la grabación y reproducción de sonido en el NAO, permitiendo implementar sistemas de reconocimiento y respuesta auditiva.
   - **USB_CAM** proporciona compatibilidad con las cámaras del nao, facilitando la integración de visión computacional en ROS 2.
   - Ambos paquetes amplían la capacidad de percepción del robot, permitiendo el desarrollo de aplicaciones más completas.
  
9. **SYNC**
   - Script desarrollado por [**Kenji Brameld (ijnek)**](https://github.com/ijnek/sync) que permite sincronizar un robot, como Nao, con un **workspace precompilado** de ROS 2 en otro pc. Su propósito principal es resolver el problema de la **compilación lenta en el Nao**, ya que el robot tiene **recursos computacionales limitados** y tarda mucho en compilar paquetes complejos.

#### Créditos y Reconocimiento

El desarrollo de **OAN** ha sido posible gracias al esfuerzo de múltiples colaboradores, con especial reconocimiento a **Antonio Bono (antbono)** y **Kenji Brameld (ijnek)**. 

- **Antonio Bono (antbono)** ha liderado la integración de **NAO con ROS 2**, estableciendo una base sólida para el Human Robot Interaction (HNI) en el NAO.
- **Kenji Brameld (ijnek)** ha realizado contribuciones esenciales para la locomoción y la sincronización de workspaces.

El trabajo conjunto de estos desarrolladores ha permitido que **OAN** se convierta en una plataforma confiable y robusta para la investigación y desarrollo del NAO en **ROS 2**. Gracias a estas contribuciones, **OAN** es actualmente una de las plataformas más completas para trabajar con el **NAO en ROS 2**.


<div id='configuración-de-nav2' />

### 🚀 Configuración de Nav2  
(Guía paso a paso sobre la instalación y ajuste de Nav2...)  

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
