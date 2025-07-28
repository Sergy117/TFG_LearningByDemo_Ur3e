Aprendizaje por Demostración (LfD): Control de Brazo Robótico UR3e mediante Mapeo Cinemático desde Pose Humana

📖 Descripción General

Este repositorio contiene el código fuente, la documentación y las herramientas para un Trabajo de Fin de Grado (TFG) centrado en la implementación de un sistema de Aprendizaje por Demostración (Learning from Demonstration - LfD) para un brazo robótico UR3e.

El núcleo del proyecto es el desarrollo de un pipeline robusto que traduce los movimientos de un operador humano, capturados por un sistema de visión estéreo, a trayectorias en el espacio articular del robot. Estas trayectorias se utilizan para entrenar un modelo generativo de la tarea basado en Primitivas de Movimiento Dinámico (DMPs). El resultado es un sistema donde el robot puede aprender y reproducir de forma autónoma habilidades motoras complejas a partir de la observación.

(Sugerencia: ¡Añade aquí un GIF o una imagen del resultado final!)
![GIF de demostración del proyecto](documentation/demo.gif)

✨ Características Principales

    Aprendizaje por Demostración (LfD) con DMPs: El sistema aprende habilidades motoras a partir de múltiples demostraciones humanas, generando trayectorias suaves y generalizables.

    Mapeo Cinemático Humano-Robot: Un "traductor" cinemático robusto que convierte la pose humana 3D a ángulos de articulación del robot, resolviendo las diferencias morfológicas.

    Estimación de Pose Humana 3D: Implementa MediaPipe Pose sobre un sistema de visión estéreo para una detección precisa de 33 landmarks corporales.

    Detección de Objetos Personalizada: Integra un modelo YOLOv8 entrenado a medida (exportado a ONNX) para la percepción de objetos clave en la tarea.

    Control del Robot en ROS: Se integra con ROS Noetic y MoveIt! para el control seguro del robot UR3e en el espacio de articulaciones, tanto en simulación como en hardware real.

🏗️ Arquitectura del Sistema

El proyecto sigue una arquitectura modular donde cada componente tiene una responsabilidad clara. El flujo de datos es el siguiente:

Cámaras Estéreo → 1. Nodo de Visión (Captura de Demos) → 2. Pipeline de Aprendizaje Offline (Traducción y Entrenamiento de DMPs) → 3. Nodo de Control del Robot (Carga y Ejecución de DMPs) → Robot UR3e

    Nodo de Visión (vision_system/): Captura las demostraciones humanas y las guarda como trayectorias 3D en archivos .csv.

    Pipeline de Aprendizaje (learning_and_validation/): Es el corazón del sistema LfD. Carga las demostraciones, las traduce al espacio articular del robot y entrena los modelos DMP, guardándolos como archivos .pkl.

    Nodo de Control (robot_control/): Carga los DMPs ya entrenados y los utiliza para generar y ejecutar la tarea de forma autónoma en el robot.

📁 Estructura del Repositorio

TFG_UR3e_Teleoperation/
|
├── 📄 README.md              # <-- Este archivo. La portada del proyecto.
├── 📄 requirements.txt        # <-- Dependencias principales de Python.
|
├── 🤖 robot_control/
│   └── src/                 # Scripts de ROS para controlar el robot (ej. dmpplayer.py)
|
├── 👁️ vision_system/
│   └── src/                 # Scripts para la captura de demostraciones (ej. main_script_final.py)
|
├── 🎓 learning_and_validation/
│   └── src/                 # Scripts para validar datos y entrenar los DMPs (ej. learn_dmp.py)
|
├── 🛠️ auxiliary_tools/
│   ├── 1_camera_calibration/  # Scripts y README para la calibración estéreo.
│   ├── 2_hand_eye_calibration/ # Scripts y README para la calibración cámara-robot.
│   └── 3_yolo_training/     # Scripts y README para el entrenamiento del modelo YOLO.
|
├── 📂 data/
│   ├── calibration_files/   # Archivos .npy de las calibraciones.
│   └── learned_dmps/        # Archivos .pkl de los DMPs aprendidos.
│
└── 📝 documentation/
    └── (Imágenes, GIFs y otros recursos para la documentación del TFG).

🚀 Guía de Inicio Rápido

Para replicar el proyecto, se debe seguir un proceso ordenado. Cada subcarpeta contiene su propio README.md con instrucciones detalladas.

    Clonar el Repositorio:
    Bash

    git clone [URL-de-tu-repositorio]
    cd TFG_UR3e_Teleoperation

    Instalar Dependencias: Instala las librerías necesarias especificadas en los diferentes archivos requirements.txt.

    Calibración (Ver auxiliary_tools/):

        Paso 1: Ejecuta el script de 1_camera_calibration.

        Paso 2: Ejecuta los scripts de 2_hand_eye_calibration.

    Entrenamiento del Detector de Objetos (Ver auxiliary_tools/):

        Sigue las instrucciones en 3_yolo_training/README.md para entrenar tu modelo .onnx.

    Creación del Dataset de Demostraciones (Ver vision_system/):

        Ejecuta el script principal de visión para grabar un conjunto de demostraciones.

    Aprendizaje del Modelo (Ver learning_and_validation/):

        Usa el script de validación para inspeccionar la calidad de tus demostraciones.

        Ejecuta el script learn_dmp.py para procesar el dataset y entrenar los DMPs.

    Ejecución en el Robot (Ver robot_control/):

        Lanza la simulación en RViz o la conexión con el robot real.

        Ejecuta el script robot_player_from_dmp.py para que el robot reproduzca la tarea aprendida.

👤 Autor

Sergio González Rodríguez

    Email: meanssergy@gmail.com

    GitHub: Sergy117

Trabajo de Fin de Grado para la Universidade de Santiago de Compostela (USC) - 2025.
