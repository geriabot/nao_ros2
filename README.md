# nao_ros2

![nao_ros2_logo](https://github.com/user-attachments/assets/7993fa06-eb04-4bc7-bc2d-a3a16c6948d7)

## 📌 Índice  
- [Introducción](#introducción)  
- [Instalación y Configuración](#instalación-y-configuración)  
  - [Requisitos Previos](#requisitos-previos)  
  - [Instalación de Ubuntu en Nao](#instalación-de-ubuntu-en-nao)  
  - [Instalación de ROS 2](#instalación-de-ros-2)  
  - [Configuración de Nav2](#configuración-de-nav2)  
- [Integración con API Web](#integración-con-api-web)  
- [Solución de Problemas y Buenas Prácticas](#solución-de-problemas-y-buenas-prácticas)  
- [Contribuciones](#contribuciones)  
- [Licencia](#licencia)  

---

## Introducción  

El objetivo de este repositorio es documentar de manera clara y unificada el proceso de instalación y configuración de **Ubuntu 22.04** y **ROS 2** en el robot **Nao**. Actualmente, la información sobre este procedimiento está altamente distribuida y mal documentada, lo que dificulta su implementación. **nao_ros2** busca solucionar este problema proporcionando una guía completa y estructurada.

Este proyecto se centra en integrar **Nao** con **Nav2**, permitiendo la navegación autónoma en un entorno mapeado y controlado a través de una API web externa.  

A lo largo de este repositorio, se detallarán los pasos necesarios para:  
- Instalar y configurar **Ubuntu 22.04** en el robot Nao.  
- Instalar **ROS 2** y los paquetes esenciales para el sistema.  
- Configurar **Nav2** para la navegación autónoma del robot.  
- Establecer comunicación con una API web para recibir comandos de destino y ejecutar movimientos precisos en el mapa.  

Este repositorio está dirigido a desarrolladores, investigadores y entusiastas de la robótica que deseen modernizar y optimizar las capacidades del robot **Nao** utilizando tecnologías de código abierto.  

---

## 🔧 Instalación y Configuración  

### 📌 Requisitos Previos  
Antes de comenzar, asegúrate de contar con:  
- Un robot **Nao V6** compatible.  
- Un ordenador con **Ubuntu 22.04** (Todavía no está probado en Ubuntu 24.04).  
- Conexión a **Internet**, un router con **tabla de IP accesible** y un **cable Ethernet**.
- **Usb drive** de al menos **4GB**.
- Conocimientos básicos de **Linux, ROS 2 y navegación robótica**.  

### 🖥 Instalación de Ubuntu en Nao  

Para instalar Ubuntu en el robot Nao V6, se debe seguir un proceso que involucra la **creación de una imagen de Ubuntu 22.04 a partir de una imagen oficial de NaoQi** (el sistema operativo predeterminado del Nao) y su posterior flasheo en el robot. Es **imprescindible por ahora utilizar la versión 2.8.5.10 de NaoQi**, ya que otras versiones pueden causar problemas de compatibilidad.

---

## **📈 Pasos para la creación de la imagen de Ubuntu**  

1. **Clonar el repositorio NaoImage**  
   ```bash
   git clone https://github.com/NaoDevils/NaoImage
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
   - Flashear la **imagen oficial** (e.g. `nao-2.8.5.10.opn`), **!!No la generada todavía!!** en el USB usando la herramienta **NaoFlasher** con Factory Reset activado.
   - Este paso es esencial ya que el firmware de los motores es distinto en la última versión (que suele venir instalada de fábrica en el Nao), y si se salta este paso los motores no funcionarán correctamente.

6. **Realizar el primer flasheo con la imagen original de NaoQi**  
   - Apagar el Nao.  
   - Insertar el **USB con el firmware original** en el robot.  
   - Mantener presionado el botón del pecho durante **6 segundos o más** hasta que parpadee la luz del pecho se ponga azul.  
   - Esperar unos minutos, el botón cambia de color a blanco y se inicia el sistema. Confirmar que NaoQi está instalado fijándose en su comportamiento (habla, se mueve, pide conectarse a la red, cambia de colores a verde a veces, reacciona a toques en el pecho, etc).
   - Se puede confirmar que la versión correcta de Naoqi (2.8.5.10) ha sido instalada a través de ssh.

7. **Crear un USB con la imagen de Ubuntu**  
   - Formatear el USB de **4GB o más** en **FAT32**.  
   - Ahora sí, flashear la imagen de **Ubuntu 22.04 generada** (`image.opn`) en el USB utilizando el flasher, con Factory Reset activado. 

8. **Flashear la imagen de Ubuntu en Nao**  
   - Apagar el robot nuevamente.  
   - Insertar el **USB con la imagen de Ubuntu**.  
   - Mantener presionado el botón del pecho durante **6 segundos o más** hasta que parpadee azul rápidamente.  
   - Esperar unos minutos hasta que el botón del pecho cambie a blanco y los LEDs se estabilicen.  

---

## **📈 Verificación de instalación exitosa**  

👌 **NaoQi instalado correctamente (antes de Ubuntu):**  
- El robot **habla, se mueve** y reacciona al tacto.  
- Cambia de colores **(verde, azul, blanco, etc.)**.  
- Solicita conexión a la red.  

👌 **Ubuntu instalado correctamente:**  
- El robot **no habla ni se mueve** automáticamente.  
- Los LEDs de los ojos y el pecho se quedan **en blanco**.  
- Las orejas del robot quedan **en azul fijo**.  

En este punto, Ubuntu ya está instalado y el robot está listo para la configuración de red y la instalación de **ROS 2**. 🚀



### ⚙️ Instalación de ROS 2  
(Pasos para instalar ROS 2 en el robot...)  

### 🚀 Configuración de Nav2  
(Guía paso a paso sobre la instalación y ajuste de Nav2...)  

---

## 🌐 Integración con API Web  
Explicación de cómo el robot se comunica con la API para recibir comandos de navegación.  

---

## 🛠 Solución de Problemas y Buenas Prácticas  
Lista de errores comunes, cómo solucionarlos y recomendaciones para optimizar el sistema.  

---

## 🤝 Contribuciones  
Si quieres mejorar este proyecto, revisa nuestra [Guía de Contribución](#).  

---

## 📜 Licencia  
Este proyecto está licenciado bajo **MIT License**. Consulta el archivo [LICENSE](#) para más detalles.  
