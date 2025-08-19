# app.py (VERSIÓN CORREGIDA)
#!/usr/bin/env python
from importlib import import_module
import os
from flask import Flask, render_template, Response, send_from_directory
from flask_cors import *
from camera_opencv import Camera
import threading

app = Flask(__name__)
CORS(app, supports_credentials=True)

# Variable global para la cámara, pero se inicializa como None
camera = None

def get_camera():
    """
    Función Singleton para asegurar que solo haya una instancia de la cámara.
    La crea la primera vez que se la necesita.
    """
    global camera
    if camera is None:
        print("Creando la instancia del objeto Camera por primera vez...")
        camera = Camera()
    return camera

def gen():
    """Generador de streaming de vídeo."""
    cam = get_camera() # Obtiene o crea la instancia de la cámara aquí
    frames_generator = cam.get_frame()
    while True:
        frame = next(frames_generator)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Ruta del streaming de vídeo."""
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- El resto de las rutas no cambian ---
dir_path = os.path.dirname(os.path.realpath(__file__))
@app.route('/api/img/<path:filename>')
def sendimg(filename):
    return send_from_directory(dir_path+'/dist/img', filename)
@app.route('/js/<path:filename>')
def sendjs(filename):
    return send_from_directory(dir_path+'/dist/js', filename)
@app.route('/css/<path:filename>')
def sendcss(filename):
    return send_from_directory(dir_path+'/dist/css', filename)
@app.route('/api/img/icon/<path:filename>')
def sendicon(filename):
    return send_from_directory(dir_path+'/dist/img/icon', filename)
@app.route('/fonts/<path:filename>')
def sendfonts(filename):
    return send_from_directory(dir_path+'/dist/fonts', filename)
@app.route('/<path:filename>')
def sendgen(filename):
    return send_from_directory(dir_path+'/dist', filename)
@app.route('/')
def index():
    return send_from_directory(dir_path+'/dist', 'index.html')

class webapp:
    def __init__(self):
        # La cámara se inicializará la primera vez que se llame a una de sus funciones
        pass

    def modeselect(self, modeInput):
        get_camera().modeSelect = modeInput

    def colorFindSet(self, H, S, V):
        get_camera().colorFindSet(H, S, V)

    def thread(self):
        app.run(host='0.0.0.0', threaded=True)

    def startthread(self):
        fps_threading=threading.Thread(target=self.thread)
        fps_threading.setDaemon(False)
        fps_threading.start()
