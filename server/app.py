#!/usr/bin/env python3
# Nombre del archivo: app.py
# Descripción: Este script implementa un servidor web basado en Flask diseñado para realizar la transmisión de video en tiempo real (streaming).

import os
# Importación de Flask y sus componentes para la gestión del servidor web, respuestas HTTP y envío de archivos.
from flask import Flask, Response, send_from_directory
# Importación de CORS para permitir solicitudes de recursos cruzados, facilitando la conexión desde clientes externos.
from flask_cors import CORS
# Importación de la clase Camera, responsable de la gestión del hardware de la cámara y la obtención de imágenes.
from camera_opencv import Camera

# Inicialización de la aplicación Flask.
app = Flask(__name__)
# Configuración de CORS para permitir credenciales y accesos desde distintos orígenes.
CORS(app, supports_credentials=True)

def gen():
    """
    Función generadora para el flujo de video.
    
    Esta función se encarga de obtener continuamente los fotogramas capturados por la cámara.
    Instancia la clase Camera (que sigue el patrón Singleton para asegurar una única instancia activa)
    y formatea cada imagen como una parte de una respuesta multipart (MIME type: multipart/x-mixed-replace).
    Esto permite que el navegador o cliente actualice la imagen constantemente, creando el efecto de video.
    """
    camera = Camera()
    for frame in Camera.frames():
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """
    Ruta para el feed de video.
    
    Endpoint: /video_feed
    Método: GET
    
    Esta función maneja las solicitudes a la ruta '/video_feed'. Retorna una instancia de la clase Response de Flask
    que encapsula el generador 'gen()'. Se define el tipo MIME como 'multipart/x-mixed-replace' con un límite 'frame',
    lo cual es estándar para transmisiones de video MJPEG (Motion JPEG).
    """
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Gestión de Rutas Estáticas y Página Principal ---
# Obtención de la ruta absoluta del directorio donde se encuentra este script.
dir_path = os.path.dirname(os.path.realpath(__file__))

@app.route('/')
def index():
    """
    Ruta raíz de la aplicación.
    
    Endpoint: /
    Método: GET
    
    Devuelve un mensaje de texto simple indicando que el servidor de video está operativo.
    Sirve como comprobación básica de conectividad.
    """
    return "Servidor de Vídeo PiCar-B Activo. Para visualizar la interfaz, utilice el cliente Python GUI.py."

if __name__ == '__main__':
    """
    Bloque principal de ejecución.
    
    Inicia el servidor Flask si el script se ejecuta directamente.
    Parámetros:
    - host='0.0.0.0': Permite que el servidor sea accesible desde cualquier dirección IP en la red local.
    - port=5000: Puerto estándar para el servicio web.
    - debug=False: El modo de depuración está desactivado para producción.
    - threaded=True: Habilita el manejo de múltiples solicitudes en hilos separados para no bloquear el video.
    """
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)