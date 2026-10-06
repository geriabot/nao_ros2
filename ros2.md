# Instalación de ROS 2 en el Nao y un PC 

Para que el robot **Nao** pueda operar correctamente con ROS 2, se recomienda instalar la distribución **ROS 2 Humble Hawksbill**.

ROS 2 Humble deberá ser instalado tanto en un pc con **Ubuntu 22.04** como en el robot. La diferencia clave es que la instalación en el **Nao** deberá de ser una instalación base, ya que no tiene interfaz gráfica.

---

## **🛠️ Pasos para instalar ROS 2 Humble en el PC y Nao**  

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

Ahora, agregar el repositorio de ROS 2 (el paquete `ros2-apt-source` instala la clave GPG y el source de apt, y los mantiene actualizados):
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb
```

---

### **2. Instalar ROS 2 Humble**  
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

Instalar ROS 2 Humble en el **Nao** (versión base, sin entorno gráfico):
```bash
sudo apt install ros-humble-ros-base
```
Este paquete incluye:
- Herramientas básicas de **ROS 2**
- Librerías de comunicación
- Soporte para **nodos, servicios y mensajes**

Instalar ROS 2 Humble en el **PC** (version de escritorio, con entorno gráfico)
```bash
sudo apt install ros-humble-desktop
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
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
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

Si ambos comandos muestran información correctamente, la instalación de ROS 2 Humble ha sido exitosa.
