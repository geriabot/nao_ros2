# Configuración y lanzamiento de SLAM y navegación autónoma

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
