# Instalación de Ubuntu 22.04 en Nao

<div id='requisitos-previos' />

## 📌 Requisitos Previos  
Antes de comenzar, asegúrate de contar con:  
- Un robot **Nao V6** compatible.  
- Un ordenador con **Ubuntu 22.04** (Todavía no está probado en Ubuntu 24.04).  
- Conexión a **Internet**, un router con **tabla de IP accesible** y un **cable Ethernet**.
- **Usb drive** de al menos **4GB**.
- Conocimientos básicos de **Linux, ROS 2 y navegación robótica**.  

---

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

## **📈 *Flasheo* de la imagen en el robot Nao**  

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

## **✅ Verificación de ls instalación**  

👌 **NaoQi instalado correctamente (antes de Ubuntu):**  
- El robot **habla, se mueve** y reacciona al tacto.  
- Cambia de colores **(verde, azul, blanco, etc.)**.  
- Solicita conexión a la red.  

👌 **Ubuntu instalado correctamente:**  
- El robot **no habla ni se mueve** automáticamente.  
- Los LEDs de los ojos y el pecho se quedan **en blanco**.  
- Las orejas del robot quedan **en azul fijo**.  

En este punto, Ubuntu ya está instalado y el robot está listo para la configuración inicial y la instalación de **ROS 2**. 🚀
