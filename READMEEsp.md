Control de Brazo Robótico UR3e mediante Mapeo Cinemático desde Pose Humana

📖 Descripción General

Este repositorio contiene el código fuente, la documentación y las herramientas para el Trabajo de Fin de Grado (TFG) centrado en la teleoperación de un brazo robótico UR3e en el espacio de articulaciones. El sistema traduce los movimientos de un operador humano, capturados por un sistema de visión estéreo con MediaPipe, a comandos de joint states para el robot, evitando deliberadamente el uso de la cinemática inversa (IK) en tiempo real para priorizar la velocidad y la eficiencia computacional.

El objetivo principal es desarrollar y validar un pipeline de mapeo cinemático robusto que sirva como base para tareas de Aprendizaje por Demostración (LfD), utilizando Primitivas de Movimiento Dinámico (DMPs) para que el robot aprenda y reproduzca tareas complejas demostradas por un humano.

(Sugerencia: ¡Añade aquí un GIF o una imagen del resultado final, como la de la comparación de Matplotlib o un vídeo de la simulación en RViz!)
![GIF de demostración del proyecto](documentation/demo.gif)

✨ Características Principales

    Sistema de Visión Estéreo: Utiliza dos cámaras Intel RealSense para una reconstrucción 3D precisa de la escena.

    Estimación de Pose Humana 3D: Implementa MediaPipe Pose para la detección de 33 landmarks corporales en tiempo real.

    Detección de Objetos Personalizada: Integra un modelo YOLOv8 entrenado a medida y exportado a formato ONNX para la detección de las piezas de la tarea.

    Mapeo Cinemático Humano-Robot: Desarrolla un "traductor" que convierte la pose humana en ángulos de articulación del robot, resolviendo las diferencias morfológicas y de sistemas de coordenadas.

    Aprendizaje por Demostración (LfD): Usa Primitivas de Movimiento Dinámico (DMPs) para aprender trayectorias complejas a partir de múltiples demostraciones humanas.

    Control del Robot en ROS: Se integra con ROS Noetic y MoveIt! para el control seguro del robot UR3e, tanto en simulación (Gazebo/RViz) como en el hardware real.

🏗️ Arquitectura del Sistema

El proyecto sigue una arquitectura modular donde cada componente tiene una responsabilidad clara. El flujo de datos es el siguiente:

Cámaras Estéreo → 1. Nodo de Visión (Captura y Triangulación 3D) → 2. Pipeline de Aprendizaje Offline (Traducción y Entrenamiento de DMPs) → 3. Nodo de Control del Robot (Carga y Ejecución de DMPs) → Robot UR3e

    Nodo de Visión (vision_system/): Captura las imágenes, detecta los landmarks humanos y los objetos, y guarda las trayectorias 3D de la demostración en archivos .csv.

    Pipeline de Aprendizaje (learning_and_validation/): Carga las demostraciones, las traduce al espacio articular del robot usando la metodología del informe técnico, y entrena los modelos DMP, guardándolos como archivos .pkl.

    Nodo de Control (robot_control/): Carga los DMPs aprendidos y los utiliza para generar y ejecutar trayectorias en el robot a través de MoveIt.

📁 Estructura del Repositorio

TFG_UR3e_Teleoperacion/
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

Para replicar el proyecto, se debe seguir un proceso ordenado. Cada subcarpeta en auxiliary_tools y los directorios principales contienen su propio README.md con instrucciones detalladas.

    Clonar el Repositorio:
    Bash

    git clone [URL-de-tu-repositorio]
    cd TFG_UR3e_Teleoperacion

    Instalar Dependencias: Instala las librerías necesarias especificadas en los diferentes archivos requirements.txt.

    Calibración (Carpeta: auxiliary_tools/):

        Paso 1: Ejecuta el script de 1_camera_calibration para calibrar el sistema estéreo.

        Paso 2: Ejecuta los scripts de 2_hand_eye_calibration para obtener la transformación entre la cámara y el robot.

    Entrenamiento del Detector de Objetos (Carpeta: auxiliary_tools/):

        Sigue las instrucciones en 3_yolo_training/README.md para capturar imágenes, etiquetarlas y entrenar tu modelo .onnx.

    Creación del Dataset de Demostraciones (Carpeta: vision_system/):

        Ejecuta el script principal de visión para grabar un conjunto de demostraciones de la tarea.

    Aprendizaje del Modelo (Carpeta: learning_and_validation/):

        Usa el script de validación para inspeccionar la calidad de tus demostraciones.

        Ejecuta el script learn_dmp.py para procesar el dataset y entrenar los DMPs.

    Ejecución en el Robot (Carpeta: robot_control/):

        Lanza la simulación en RViz o la conexión con el robot real.

        Ejecuta el script robot_player_from_dmp.py para que el robot reproduzca la tarea aprendida.

👤 Autor

Sergio González Rodríguez

    Email: meanssergy@gmail.com

    GitHub: Sergy117

Trabajo de Fin de Grado para la Universidade de Santiago de Compostela (USC) - 2025.