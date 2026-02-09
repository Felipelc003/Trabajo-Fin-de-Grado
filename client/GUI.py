#!/usr/bin/python3
#
# This file is part of  Diseño Modular e Incremental para la 
# Navegación Autónoma Contextual en Sistemas Embebidos de Bajo Coste
#
# Copyright 2026 Felipe López Castro <i12locaf@uco.es>
#
# Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste is free software: you can redistribute it 
# and/or modify it under the terms of the GNU General Public License 
# as published by the Free Software Foundation, either version 3 of the License, 
# or  (at your option) any later version.
# 
# Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste.
# If not, see <https://www.gnu.org/licenses/>.
#

# -*- coding: UTF-8 -*-
# File name   : GUI.py
# Description : Cliente Gráfico (PC Host) para control remoto del PiCar-B.
#               Desarrollado con Tkinter (Interfaz) y OpenCV (Visualización).
#               Características:
#               - Comunicación bidireccional vía WebSocket (Control + Telemetría).
#               - Streaming de video MJPEG desde el servidor Flask.
#               - Controles manuales (Teclado/Botones) para Motores y Cámara.
#               - Herramientas de depuración de Visión (Ajuste HSV en tiempo real).

import time
import threading
import tkinter as tk
import asyncio
import websockets.protocol
import json
import cv2
import numpy as np

# --- Constantes de Estilo (Tema Oscuro) ---
color_bg='#000000'      # Fondo negro
color_text='#E1F5FE'    # Texto claro (Azul muy pálido)
color_btn='#0277BD'     # Botones azules (Material Design)

# =================== Estado Global del Cliente ===================

# Gestión de botones activos (exclusión mutua para modos autónomos)
function_button_active = None 

# Estados de control manual (para evitar reenvío de comandos repetidos)
steering_state = 'center'
camera_pan_state = 'stop'
camera_tilt_state = 'stop'
ip_stu = 1  # Estado de la conexión (1=Desconectado, 0=Conectado)

# Variables Reactivas de Tkinter (Data Binding)
cpu_temp_var = None
cpu_use_var = None
ram_use_var = None
var_Speed = None 

# Control del hilo de visualización de video
video_thread = None
video_stop_event = None

# --- Estado del Modo de Calibración HSV (Visión) ---
# Permite ajustar los rangos de color desde el cliente y ver la máscara resultante.
hsv_mode = False
hsv_windows_created = False
hsv_init = {"H_min": 0, "S_min": 0, "V_min": 0, "H_max": 179, "S_max": 255, "V_max": 255}

# --- Infraestructura de Red ---
websocket = None
event_loop = None

def reset_client_state():
    """Restablece los estados internos de la interfaz al conectar/desconectar."""
    global function_button_active, steering_state, camera_pan_state, camera_tilt_state
    global hsv_mode, hsv_windows_created
    
    function_button_active = None
    steering_state = 'center'
    camera_pan_state = 'stop'
    camera_tilt_state = 'stop'
    
    # Reiniciar herramientas de visión
    hsv_mode = False
    if hsv_windows_created:
        _hsv_destroy_windows()
    
    print("[GUI] ✓ Estados reiniciados")

# =================== Herramientas de Depuración de Visión (HSV) ===================

def _hsv_create_windows():
    """Crea ventanas de OpenCV con Sliders (Trackbars) para calibración de color."""
    global hsv_windows_created
    if hsv_windows_created: return
    try:
        cv2.namedWindow("Controls")
        cv2.resizeWindow("Controls", 500, 300)
        # Sliders para rango inferior (Min)
        cv2.createTrackbar("H_min", "Controls", hsv_init["H_min"], 179, lambda x: None)
        cv2.createTrackbar("S_min", "Controls", hsv_init["S_min"], 255, lambda x: None)
        cv2.createTrackbar("V_min", "Controls", hsv_init["V_min"], 255, lambda x: None)
        # Sliders para rango superior (Max)
        cv2.createTrackbar("H_max", "Controls", hsv_init["H_max"], 179, lambda x: None)
        cv2.createTrackbar("S_max", "Controls", hsv_init["S_max"], 255, lambda x: None)
        cv2.createTrackbar("V_max", "Controls", hsv_init["V_max"], 255, lambda x: None)
        hsv_windows_created = True
    except Exception as e:
        print(f"[HSV] Error creando ventanas: {e}")

def _hsv_destroy_windows():
    """Cierra las ventanas de depuración."""
    global hsv_windows_created
    if not hsv_windows_created: return
    try:
        cv2.destroyWindow("Controls")
        cv2.destroyWindow("Mask")
    except: pass
    hsv_windows_created = False

def _hsv_get_range():
    """Lee los valores actuales de los sliders."""
    if not hsv_windows_created: return (0,0,0,179,255,255)
    return (cv2.getTrackbarPos("H_min", "Controls"), cv2.getTrackbarPos("S_min", "Controls"),
            cv2.getTrackbarPos("V_min", "Controls"), cv2.getTrackbarPos("H_max", "Controls"),
            cv2.getTrackbarPos("S_max", "Controls"), cv2.getTrackbarPos("V_max", "Controls"))

def _hsv_enable(flag: bool):
    """Activa el modo de visualización de máscara binaria."""
    global hsv_mode
    hsv_mode = bool(flag)

# =================== Comunicación WebSocket ===================

def send_command(command):
    """Envía un comando de texto al servidor robot."""
    if websocket and websocket.state == websockets.protocol.State.OPEN:
        asyncio.run_coroutine_threadsafe(websocket.send(str(command)), event_loop)

# =================== Streaming de Video ===================

def show_video_stream(ip_address, stop_event):
    """
    Hilo dedicado a la captura y visualización del stream MJPEG.
    Se conecta al servidor Flask (puerto 5000) y muestra frames con OpenCV.
    """
    video_url = f"http://{ip_address}:5000/video_feed"
    window_name = "Stream"
    window_created = False
    cap = None

    while not stop_event.is_set():
        # Lógica de reconexión automática
        if cap is None or not cap.isOpened():
            try:
                if cap: cap.release()
                cap = cv2.VideoCapture(video_url)
            except: cap = None
            time.sleep(0.5)
            continue

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.2); cap.release(); cap = None
            continue

        if not window_created:
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
            window_created = True

        # Procesamiento de imagen en cliente (solo visualización)
        if hsv_mode:
            if not hsv_windows_created: _hsv_create_windows()
            hm, sm, vm, hM, sM, vM = _hsv_get_range()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([hm, sm, vm]), np.array([hM, sM, vM]))
            cv2.imshow("Mask", mask)
        else:
            if hsv_windows_created: _hsv_destroy_windows()

        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == 27: # Tecla ESC para salir
            stop_event.set(); break

    if cap: cap.release()
    try: cv2.destroyAllWindows()
    except: pass
    _hsv_destroy_windows()

# =================== Lógica de Red y Telemetría ===================

def request_system_info():
    """Solicita periódicamente (Polling) información del sistema al servidor."""
    if websocket:
        send_command('get_info')
        # Programar próxima llamada en 2000ms (2 segundos)
        root.after(2000, request_system_info)

async def network_loop(ip_address, port):
    """
    Bucle principal asíncrono de red.
    Maneja la conexión WebSocket y la recepción de mensajes.
    """
    global websocket, event_loop, ip_stu
    event_loop = asyncio.get_running_loop()
    uri = f"ws://{ip_address}:{port}"
    try:
        async with websockets.connect(uri) as ws:
            websocket = ws
            print("Conectado al servidor!")
            reset_client_state()
            
            # Actualización de UI
            l_ip_4.config(text='Connected', bg='#558B2F'); l_ip_5.config(text=f'IP:{ip_address}')
            E1.config(state='disabled'); Btn_Connect.config(state='disabled'); ip_stu=0
            
            # Persistencia de última IP
            with open("last_ip.txt", "w") as f: f.write(ip_address)

            # Iniciar hilo de video
            global video_thread, video_stop_event
            video_stop_event = threading.Event()
            video_thread = threading.Thread(target=show_video_stream, args=(ip_address, video_stop_event), daemon=True)
            video_thread.start()
            
            # Autenticación
            await websocket.send("admin:123456")
            request_system_info()
            
            # Bucle de recepción de mensajes
            while True:
                message = await websocket.recv()
                try:
                    data = json.loads(message)
                    # Procesamiento de Telemetría
                    if data.get('title') == 'info_update':
                        info = data.get('data', {})
                        temp = info.get('CPU_Temp') or info.get('cpu_temp', 'N/A')
                        cpu = info.get('CPU_Usage') or info.get('cpu_usage', info.get('cpu_use', 'N/A'))
                        ram = info.get('RAM_Usage') or info.get('ram_usage', info.get('ram_use', 'N/A'))
                        
                        cpu_temp_var.set(f"{temp}")
                        cpu_use_var.set(f"{cpu}")
                        ram_use_var.set(f"{ram}")

                except Exception as e:
                    print(f"Error procesando mensaje: {e}")

    except Exception as e:
        print(f"Error de conexión: {e}")
    finally:
        # Limpieza al desconectar
        websocket = None
        if video_stop_event: video_stop_event.set()
        ip_stu = 1
        E1.config(state='normal'); Btn_Connect.config(state='normal')
        l_ip_4.config(text='Disconnected', bg='#F44336'); l_ip_5.config(text='<No IP>')
        cpu_temp_var.set("N/A"); cpu_use_var.set("N/A"); ram_use_var.set("N/A")

def start_network_thread():
    """Lanzador del hilo de red desde el botón Conectar."""
    ip_adr = E1.get().strip()
    if not ip_adr: return
    l_ip_4.config(text='Connecting', bg='#FF8F00')
    threading.Thread(target=lambda: asyncio.run(network_loop(ip_adr, 8888)), daemon=True).start()

def connect(event):
    if ip_stu == 1: start_network_thread()

# =================== Callback de Controles UI ===================

def speed_send(event):
    """Envía la actualización de velocidad cuando se mueve el slider."""
    send_command(f'Speed {var_Speed.get()}')

# --- Control de Cámara (Pan/Tilt) ---
def call_up(event):
    global camera_tilt_state
    if camera_tilt_state != 'up': send_command('up'); camera_tilt_state = 'up'
    else: send_command('home'); camera_tilt_state = 'stop'

def call_down(event):
    global camera_tilt_state
    if camera_tilt_state != 'down': send_command('down'); camera_tilt_state = 'down'
    else: send_command('home'); camera_tilt_state = 'stop'

def call_lookleft(event):
    global camera_pan_state
    if camera_pan_state != 'left': send_command('lookleft'); camera_pan_state = 'left'
    else: send_command('home'); camera_pan_state = 'stop'

def call_lookright(event):
    global camera_pan_state
    if camera_pan_state != 'right': send_command('lookright'); camera_pan_state = 'right'
    else: send_command('home'); camera_pan_state = 'stop'

def call_home(event): 
    """Centra la cámara y detiene servos."""
    global camera_pan_state, camera_tilt_state
    send_command('home'); camera_pan_state = 'stop'; camera_tilt_state = 'stop'

# --- Control de Dirección y Tracción ---
def call_turn_left(event):
    global steering_state
    if steering_state != 'left': send_command('left'); steering_state = 'left'
    else: send_command('TS'); steering_state = 'center'

def call_turn_right(event):
    global steering_state
    if steering_state != 'right': send_command('right'); steering_state = 'right'
    else: send_command('TS'); steering_state = 'center'

def call_forward(event): send_command('forward')
def call_backward(event): send_command('backward')
def call_stop(event): send_command('DS') # Drive Stop

# --- Gestión de Botones de Modo (Toggle) ---
def toggle_function(button_widget, start_command, stop_command):
    """Maneja el estado activo/inactivo de los botones de función."""
    global function_button_active, motor_controls, servo_controls
    
    # Desactivar botón previo visualmente
    if function_button_active and function_button_active != button_widget:
        function_button_active.config(bg=color_btn)
    
    # Si se pulsa el mismo botón activo -> Apagar
    if function_button_active == button_widget:
        send_command(stop_command)
        button_widget.config(bg=color_btn)
        function_button_active = None
        # Rehabilitar controles manuales
        for btn in motor_controls + servo_controls: btn.config(state='normal')
    
    # Si se pulsa un botón nuevo -> Encender
    else:
        send_command(start_command)
        button_widget.config(bg='#4CAF50') # Verde activo
        function_button_active = button_widget
        # Deshabilitar controles manuales en modos automáticos (salvo FindColor)
        disable_controls = (start_command != 'findColor')
        for btn in motor_controls + servo_controls: 
            btn.config(state='disabled' if disable_controls else 'normal')

def call_FindColor(event):
    toggle_function(Btn_FindColor, 'findColor', 'stopCV')
    _hsv_enable(function_button_active == Btn_FindColor)

def call_LineTrack(event):
    toggle_function(Btn_LineTrack, 'trackLine', 'pauseFunctions')


# =================== Construcción de Interfaz Gráfica (Tkinter) ===================
def loop():
    global root, cpu_temp_var, cpu_use_var, ram_use_var, var_Speed
    global E1, Btn_Connect, l_ip_4, l_ip_5
    
    root = tk.Tk(); root.title('Control'); 
    root.geometry('720x300'); # Dimensiones de ventana
    root.config(bg=color_bg)
    
    # Variables vinculadas a etiquetas de texto
    cpu_temp_var = tk.StringVar(value="N/A")
    cpu_use_var = tk.StringVar(value="N/A")
    ram_use_var = tk.StringVar(value="N/A")
    var_Speed = tk.StringVar(); var_Speed.set(100)
    
    try:
        logo=tk.PhotoImage(file='logo.png'); tk.Label(root,image=logo,bg=color_bg).place(x=30,y=13)
    except: pass

    # --- Posicionamiento de Componentes ---
    
    # 1. Panel Superior: Conexión IP y Telemetría
    connent_input(125, 15)
    information_screen(380, 15)
    
    # 2. Panel Derecho: Botones de Función Autónoma
    function_buttons(350, 205) 
    
    # 3. Panel Inferior Izquierdo: Controles de Motor
    motor_buttons(30, 115)

    # 4. Panel Inferior Derecho: Controles de Servo/Cámara
    servo_buttons(350, 115)

    # 5. Slider de Velocidad
    scale_speed(30, 230, 203)
    
    # Auto-rellenar última IP usada
    try:
        with open("last_ip.txt") as f:
            last = f.read().strip()
            if last: E1.insert(0, last)
    except: pass

    root.mainloop()

def connent_input(x,y):
    global E1, Btn_Connect
    tk.Label(root,width=10,text='IP Address:',fg=color_text,bg='#000000').place(x=x,y=y)
    E1 = tk.Entry(root,show=None,width=16,bg="#37474F",fg='#eceff1'); E1.place(x=x+5,y=y+25)
    Btn_Connect= tk.Button(root, width=8,height=2, text='Connect',fg=color_text,bg=color_btn,relief='ridge')
    Btn_Connect.place(x=x+130,y=y)
    root.bind('<Return>', connect); Btn_Connect.bind('<ButtonPress-1>', connect)

def information_screen(x,y):
    global l_ip_4, l_ip_5
    # Etiquetas estáticas
    tk.Label(root,width=10,text='CPU Temp:',fg=color_text,bg=color_bg, anchor='w').place(x=x,y=y)
    tk.Label(root,width=10,text='CPU Usage:',fg=color_text,bg=color_bg, anchor='w').place(x=x,y=y+30)
    tk.Label(root,width=10,text='RAM Usage:',fg=color_text,bg=color_bg, anchor='w').place(x=x,y=y+60)
    
    # Valores dinámicos
    tk.Label(root,width=8,textvariable=cpu_temp_var,fg=color_text,bg=color_bg, anchor='w').place(x=x+80,y=y)
    tk.Label(root,width=8,textvariable=cpu_use_var,fg=color_text,bg=color_bg, anchor='w').place(x=x+80,y=y+30)
    tk.Label(root,width=8,textvariable=ram_use_var,fg=color_text,bg=color_bg, anchor='w').place(x=x+80,y=y+60)

    # Unidades
    tk.Label(root,width=10,text='°C',fg=color_text,bg=color_bg, anchor='w').place(x=x+110,y=y)
    tk.Label(root,width=10,text=' %',fg=color_text,bg=color_bg, anchor='w').place(x=x+110,y=y+30)
    tk.Label(root,width=10,text=' %',fg=color_text,bg=color_bg, anchor='w').place(x=x+110,y=y+60)

    # Estado de conexión
    l_ip_4=tk.Label(root,width=18,text='Disconnected',fg=color_text,bg='#F44336'); l_ip_4.place(x=x+180,y=y)
    l_ip_5=tk.Label(root,width=18,text='<No IP>',fg=color_text,bg=color_btn); l_ip_5.place(x=x+180,y=y+35)

def motor_buttons(x,y):
    global motor_controls
    motor_controls = []
    
    # Definición de botones de movimiento (WASD compatible)
    btn_fwd = tk.Button(root, width=8, text='Forward',fg=color_text,bg=color_btn,relief='ridge')
    btn_fwd.place(x=x+90,y=y)
    btn_fwd.bind('<ButtonPress-1>', call_forward); btn_fwd.bind('<ButtonRelease-1>', call_stop)
    root.bind('<KeyPress-w>', call_forward); root.bind('<KeyRelease-w>', call_stop)
    motor_controls.append(btn_fwd)

    btn_bwd = tk.Button(root, width=10, text='Backward',fg=color_text,bg=color_btn,relief='ridge')
    btn_bwd.place(x=x+90,y=y+35)
    btn_bwd.bind('<ButtonPress-1>', call_backward); btn_bwd.bind('<ButtonRelease-1>', call_stop)
    root.bind('<KeyPress-s>', call_backward); root.bind('<KeyRelease-s>', call_stop)
    motor_controls.append(btn_bwd)

    btn_left = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge')
    btn_left.place(x=x,y=y+35)
    btn_left.bind('<ButtonPress-1>', call_turn_left); root.bind('<KeyPress-a>', call_turn_left)
    motor_controls.append(btn_left)

    btn_right = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge')
    btn_right.place(x=x+180,y=y+35)
    btn_right.bind('<ButtonPress-1>', call_turn_right); root.bind('<KeyPress-d>', call_turn_right)
    motor_controls.append(btn_right)

def servo_buttons(x,y):
    global servo_controls
    servo_controls = []
    
    # Definición de botones de cámara (IJKL compatible)
    btn_up = tk.Button(root, width=8, text='Up',fg=color_text,bg=color_btn,relief='ridge')
    btn_up.place(x=x+90,y=y)
    btn_up.bind('<ButtonPress-1>', call_up); root.bind('<KeyPress-i>', call_up)
    servo_controls.append(btn_up)

    btn_down = tk.Button(root, width=8, text='Down',fg=color_text,bg=color_btn,relief='ridge')
    btn_down.place(x=x+90,y=y+35)
    btn_down.bind('<ButtonPress-1>', call_down); root.bind('<KeyPress-k>', call_down)
    servo_controls.append(btn_down)

    btn_left = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge')
    btn_left.place(x=x,y=y+35)
    btn_left.bind('<ButtonPress-1>', call_lookleft); root.bind('<KeyPress-j>', call_lookleft)
    servo_controls.append(btn_left)

    btn_right = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge')
    btn_right.place(x=x+180,y=y+35)
    btn_right.bind('<ButtonPress-1>', call_lookright); root.bind('<KeyPress-l>', call_lookright)
    servo_controls.append(btn_right)
    
    btn_center = tk.Button(root, width=8, text='Center',fg=color_text,bg=color_btn,relief='ridge')
    btn_center.place(x=x+180,y=y)
    btn_center.bind('<ButtonPress-1>', call_home); root.bind('<KeyPress-h>', call_home)

def scale_speed(x,y,w):
    tk.Scale(root,label=None,from_=0,to=100,orient=tk.HORIZONTAL,length=w,showvalue=1,
             tickinterval=None,resolution=1,variable=var_Speed,troughcolor='#448AFF',
             command=speed_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)

def function_buttons(x,y):
    global Btn_FindColor, Btn_LineTrack
    
    # Botón de modo de calibración de color
    Btn_FindColor = tk.Button(root, width=12, text='Find Color',fg=color_text,bg=color_btn,relief='ridge')
    Btn_FindColor.place(x=x,y=y)
    Btn_FindColor.bind('<ButtonPress-1>', call_FindColor)
    
    # Botón de seguidor de línea autónomo
    Btn_LineTrack = tk.Button(root, width=12, text='Line Track',fg=color_text,bg=color_btn,relief='ridge')
    Btn_LineTrack.place(x=x,y=y+35)
    Btn_LineTrack.bind('<ButtonPress-1>', call_LineTrack)

if __name__ == '__main__':
    loop()