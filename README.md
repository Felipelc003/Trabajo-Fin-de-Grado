# Diseño Modular e Incremental para la Navegación Autónoma Contextual en Sistemas Embebidos de Bajo Coste

Este repositorio contiene el código fuente y la documentación técnica del Trabajo Fin de Grado (TFG) desarrollado para el Grado en Ingeniería Informática de la Universidad de Córdoba (Curso 2025-2026).

## 📖 Descripción

El proyecto consiste en el diseño e implementación de un vehículo autónomo capaz de navegar en entornos contextuales utilizando visión artificial. El sistema utiliza una cámara y procesamiento de imagen para:

* **Seguimiento de líneas multicolor:** Algoritmos robustos para seguir líneas de diferentes colores (negro, blanco, amarillo, rojo).
* **Navegación contextual:** Interpretación de señales visuales y códigos QR para la toma de decisiones (giros, paradas, cambios de velocidad).
* **Control adaptativo:** Controladores PID y lógica de velocidad variable según la curvatura.
* **Arquitectura modular:** Diseño por capas (HAL, Visión, Control, Aplicación) ejecutado sobre una Raspberry Pi con multiprocesamiento.

El sistema incluye un servidor web basado en Flask y WebSockets para telemetría y control en tiempo real desde una interfaz externa.

## 🚀 Requisitos y Hardware

### Componentes Principales
* **Unidad de Procesamiento:** Raspberry Pi 3 Model B+ (o superior) con Raspberry Pi OS.
* **Sensor de Visión:** Raspberry Pi Camera Module (Interfaz CSI).
* **Actuadores:** Motor DC y Servos.
* **Expansión:** Adeept Motor HAT V2.0 (Controlador PCA9685).

## 🛠️ Tecnologías y Dependencias

El proyecto está desarrollado en **Python 3** y utiliza las siguientes librerías clave:

* **Visión Artificial:** `OpenCV` (Procesamiento de imagen), `NumPy` (Cálculo matricial), `PyZbar` (Lectura de códigos QR).
* **Control Hardware:** `RPi.GPIO`, `adafruit-circuitpython-pca9685`, `adafruit-circuitpython-motor`, `adafruit-blinka`.
* **Web y Comunicación:** `Flask`, `flask-cors`, `websockets`.

## ⚙️ Instalación y Despliegue

### 1. Preparación del Entorno
Asegúrate de tener habilitada la interfaz de cámara y I2C en la Raspberry Pi (`raspi-config`).

Instala las dependencias del sistema (especialmente OpenCV, que se recomienda instalar vía apt):
```bash
sudo apt update
sudo apt install python3-opencv libhdf5-dev libatlas-base-dev
