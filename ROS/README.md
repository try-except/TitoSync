# ROS2 Workspace Documentation

## 1. Instalación de dependencias
Se asume que se tiene instalado ROS2 Humble

Para las dependencias se recomienda usar un entorno de python si no se quieren tener problemas con otros paquetes ya instalados en el computador.

```
python3 -m venv ros2env
source ros2env/bin/activate
pip install -r requirements.txt
```

Si no consideran necesario usar el ambiente pueden simplemente:


```
pip install -r requirements.txt
```

En caso de si usar el entorno, recuerden hacer el source hace vez que lo usen:

```
source ros2env/bin/activate
```

## 2. Estructura del workspace

El workspace se organiza de la siguiente forma:

```
ros2_ws/
├── build/
├── install/
├── log/
└── src/
```

Tanto build, install y log no vendran por defecto ya que son propios de cada workspace pero se crean de manera automatica al compilar el workspace (ver seccion 5)

### build/
Contiene los resultados generados por `colcon build`.

- Subdirectorios por paquete: `coms_pkg`, `control_pkg`, `perception_pkg`

### install/
Carpeta generada tras la instalación de los paquetes.

- Contiene ejecutables, bibliotecas, configuraciones de entorno.
- Un subdirectorio por paquete.

### log/
Incluye los registros de compilación y ejecución generados por colcon y ROS2.

### src/
Código fuente del proyecto.

Incluye los siguientes paquetes:

- **coms_pkg** → Comunicación y envío de datos por serial.  
- **control_pkg** → Nodo de control del sistema.  
- **perception_pkg** → Percepción, cámara RealSense y procesamiento de centroides.


---

## 3. Paquetes del workspace


### coms_pkg

Paquete encargado de la comunicación con dispositivos externos vía puerto serial.

**Nodo:** `ComsNode` (`coms`)

**Funciones principales:**
- Suscripción al tópico `control_output` (tipo `std_msgs/String`).
- Recibe mensajes del nodo de control.
- Envía los datos por puerto serial en formato ASCII.
- Registra actividad en logs (para debug).
- Inicializa y cierra ROS2 y el puerto serial correctamente.
- En caso de errores revisar en que puerto esta conectado el cable usb con el comando

```
ls /dev/tty*
```


---

### perception_pkg

Paquete encargado de la adquisición y procesamiento de imágenes desde una cámara Intel RealSense.



**Funciones principales:**
- El archivo `calibracion_oak_final.py` permite calibrar tanto la posicion a trackear como el color del objeto de interes
- El archivo `camara_oak.py` lee los parametros definidos en el proceso de calibracion y procede a indentificar el objeto de interes para luego publicar su informacion en topicos usados por los nodos de control


Incluye un sistema de calibración totalmente interactivo.

Antes de hacer cualquier cosa en modo automatico es necesario calibrar la camara corriendo:

```
python3 calibracion_oak_final.py
```

Esto levantara una GUI donde con click izquierdo seleccionan el color de interes y con el click derecho el punto de referencia para el controlador.

---

### control_pkg

Paquete responsable del control general del sistema.

**Funciones:**
- Recibe la información procesada desde `perception_pkg`.
- El nodo `control` genera comandos de salida para `coms_pkg`.
- El nodo `keyboard_control` permite mover dos motores con las teclas asdw


---

## 4. Launch File: bringup_launch.py

Este archivo de lanzamiento inicia los tres nodos principales del proyecto:

### Nodo de percepción — camara
- Paquete: `perception_pkg`
- Nodo `camara`
- Salida: `screen`

### Nodo de control — control
- Paquete: `control_pkg`
- Procesa percepción y genera comandos de control
- Salida: `screen`

### Nodo de comunicaciones — coms
- Paquete: `coms_pkg`
- Recibe comandos desde el nodo de control y los envía por puerto serial
- Salida: `screen`

### Opciones del launch
- Argumento `screen` para habilitar/deshabilitar salida en consola.
- `emulate_tty=True` mejora el formato de salida en terminal.

### Comando de uso
```bash
ros2 launch perception_pkg bringup_launch.py

## 5. Compilar y ejecutar el workspace

### Compilar
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```


## 6. Agregar al .bashrc

Deben poner ruta donde quede instalado, ejemplo: 
``
source ~/TitoSync/ros2_ws/install/setup.bash
``

## 7. Como ejecutar nodos individuales

```
ros2 run perception_pkg camara
ros2 run control_pkg control
ros2 run control_pkg keyboard_control
ros2 run coms_pkg coms
```
