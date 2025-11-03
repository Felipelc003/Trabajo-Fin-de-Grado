#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# File name   : GUI.py
# Description : client for PiCar-B
# Website     : www.adeept.com
# Author      : William (Adaptado por Felipe y Gemini para Websockets)
# Date        : 2025/8/18

import sys
import time
import threading
import tkinter as tk
import asyncio
import websockets.protocol
import math
import os
import json
import cv2
import numpy as np

# --- Variables Globales y Configuración ---
DS_stu, TS_stu, color_bg, color_text, color_btn, color_line, color_can, color_oval, target_color = (0,)*9
speed, ip_stu, Switch_3, Switch_2, Switch_1, servo_stu, function_stu = (0,)*7
function_button_active = None 
steering_state, camera_pan_state, camera_tilt_state = 'center', 'stop', 'stop'

# --- NUEVAS VARIABLES DE TKINTER PARA LOS DATOS ---
cpu_temp_var, cpu_use_var, ram_use_var = None, None, None

# --- Estado para FindColor/HSV ---
hsv_mode = False
hsv_windows_created = False

# Valores por defecto para los sliders
hsv_init = {
    "H_min": 0, "S_min": 0, "V_min": 0,
    "H_max": 179, "S_max": 255, "V_max": 255,
}


def global_init():
    global DS_stu, TS_stu, color_bg, color_text, color_btn, color_line, color_can, color_oval, target_color
    global speed, ip_stu, Switch_3, Switch_2, Switch_1, servo_stu, function_stu
    DS_stu=0; TS_stu=0; color_bg='#000000'; color_text='#E1F5FE'; color_btn='#0277BD'; line_color='#01579B'
    color_can='#212121'; color_oval='#2196F3'; target_color='#FF6D00'; speed=1; ip_stu=1
    Switch_3=0; Switch_2=0; Switch_1=0; servo_stu=0; function_stu=0

global_init()

# --- Lógica de Websockets ---
websocket = None
event_loop = None

def _hsv_create_windows():
    global hsv_windows_created
    if hsv_windows_created:
        return
    try:
        cv2.namedWindow("Controls")
        cv2.resizeWindow("Controls", 500, 300)
        cv2.createTrackbar("H_min", "Controls", hsv_init["H_min"], 179, lambda x: None)
        cv2.createTrackbar("S_min", "Controls", hsv_init["S_min"], 255, lambda x: None)
        cv2.createTrackbar("V_min", "Controls", hsv_init["V_min"], 255, lambda x: None)
        cv2.createTrackbar("H_max", "Controls", hsv_init["H_max"], 179, lambda x: None)
        cv2.createTrackbar("S_max", "Controls", hsv_init["S_max"], 255, lambda x: None)
        cv2.createTrackbar("V_max", "Controls", hsv_init["V_max"], 255, lambda x: None)
        hsv_windows_created = True
        print("[HSV] Ventanas 'Controls' y 'Mask' listas. Ajusta los deslizadores; pulsa 'q' en la ventana de vídeo para imprimir los valores actuales.")
    except Exception as e:
        print(f"[HSV] Error creando ventanas: {e}")

def _hsv_destroy_windows():
    global hsv_windows_created
    if not hsv_windows_created:
        return
    try:
        cv2.destroyWindow("Controls")
    except Exception:
        pass
    try:
        cv2.destroyWindow("Mask")
    except Exception:
        pass
    hsv_windows_created = False
    print("[HSV] Ventanas cerradas.")

def _hsv_get_range():
    if not hsv_windows_created:
        return (0,0,0,179,255,255)
    h_min = cv2.getTrackbarPos("H_min", "Controls")
    s_min = cv2.getTrackbarPos("S_min", "Controls")
    v_min = cv2.getTrackbarPos("V_min", "Controls")
    h_max = cv2.getTrackbarPos("H_max", "Controls")
    s_max = cv2.getTrackbarPos("S_max", "Controls")
    v_max = cv2.getTrackbarPos("V_max", "Controls")
    return (h_min, s_min, v_min, h_max, s_max, v_max)

def _hsv_enable(flag: bool):
    """Solo cambia el flag. La creación/destrucción real se hace en el hilo de vídeo."""
    global hsv_mode
    hsv_mode = bool(flag)
    print(f"[HSV] Modo {'activado' if hsv_mode else 'desactivado'}.")


def send_command(command):
    if websocket and websocket.state == websockets.protocol.State.OPEN:
        asyncio.run_coroutine_threadsafe(websocket.send(str(command)), event_loop)
    else:
        print("No conectado, no se puede enviar el comando.")

def show_video_stream(ip_address):
    video_url = f"http://{ip_address}:5000/video_feed"
    print(f"📹 Intentando conectar al stream de vídeo en: {video_url}")
    cap = cv2.VideoCapture(video_url)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo recibir el fotograma. Reintentando conexión...")
            time.sleep(1)
            cap.release()
            cap = cv2.VideoCapture(video_url)
            continue
        
        if hsv_mode:
            if not hsv_windows_created:
                _hsv_create_windows()

            # Leer rango actual desde los sliders
            h_min, s_min, v_min, h_max, s_max, v_max = _hsv_get_range()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
            upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            cv2.imshow("Mask", mask)
        else:
            # Si salimos del modo HSV, cierra ventanas auxiliares
            if hsv_windows_created:
                _hsv_destroy_windows()

        cv2.imshow("PiCar-B Stream", frame)
        key = cv2.waitKey(1) & 0xFF

        if hsv_mode and key == ord('q'):
            h_min, s_min, v_min, h_max, s_max, v_max = _hsv_get_range()
            print("\n--- RANGO HSV ACTUAL ---")
            print(f"color_lower = np.array([{h_min}, {s_min}, {v_min}], dtype=np.uint8)")
            print(f"color_upper = np.array([{h_max}, {s_max}, {v_max}], dtype=np.uint8)")
            print("------------------------")

        # Salida general (si cierras el WS o pulsas la X de OpenCV)
        if key == 27 or websocket is None:  # ESC o desconexión
            break

    print("Stream de vídeo detenido.")
    cap.release()
    cv2.destroyAllWindows()

# --- NUEVA FUNCIÓN PARA PEDIR DATOS PERIÓDICAMENTE ---
def request_system_info():
    if websocket: # Solo pide si estamos conectados
        send_command('get_info')
        root.after(2000, request_system_info) # Llama a esta misma función otra vez en 2 segundos

async def network_loop(ip_address, port):
    global websocket, event_loop, ip_stu
    event_loop = asyncio.get_running_loop()
    uri = f"ws://{ip_address}:{port}"
    try:
        async with websockets.connect(uri) as ws:
            websocket = ws
            print("Conectado al servidor websocket!")
            l_ip_4.config(text='Connected', bg='#558B2F'); l_ip_5.config(text=f'IP:{ip_address}')
            E1.config(state='disabled'); Btn14.config(state='disabled'); ip_stu=0
            with open("last_ip.txt", "w") as f:
                f.write(ip_address)


            
            video_thread = threading.Thread(target=show_video_stream, args=(ip_address,), daemon=True)
            video_thread.start()
            
            await websocket.send("admin:123456")

            # --- Iniciar el bucle de petición de información ---
            request_system_info()
            
            while True:
                message = await websocket.recv()
                try:
                    data = json.loads(message)
                    if data.get('title') == 'scanResult':
                        scan_results = data.get('data', [])
                        draw_radar_scan(scan_results)
                    
                    # --- NUEVO BLOQUE PARA ACTUALIZAR LA INFO ---
                    elif data.get('title') == 'info_update':
                        info = data.get('data', {})
                        cpu_temp_var.set(f"{info.get('cpu_temp', 'N/A')} ℃")
                        cpu_use_var.set(f"{info.get('cpu_use', 'N/A')} %")
                        ram_use_var.set(f"{info.get('ram_use', 'N/A')} %")

                except (json.JSONDecodeError, TypeError):
                    print(f"Mensaje del servidor (no JSON): {message}")
                except Exception as e:
                    print(f"Error procesando mensaje: {e}")

    except Exception as e:
        print(f"Error en el bucle de red: {e}")
    finally:
        websocket = None; ip_stu = 1; l_ip_4.config(text='Disconnected', bg='#F44336')
        E1.config(state='normal'); Btn14.config(state='normal')
        cpu_temp_var.set("N/A"); cpu_use_var.set("N/A"); ram_use_var.set("N/A")
        print("Cerrando ventana de vídeo si está abierta...")

def start_network_thread():
    ip_adr = E1.get().strip()
    if not ip_adr:
        l_ip_4.config(text='Enter IP!', bg='#F44336')
        print("Error: El campo de la IP no puede estar vacío.")
        return
    l_ip_4.config(text='Connecting', bg='#FF8F00')
    threading.Thread(target=lambda: asyncio.run(network_loop(ip_adr, 8888)), daemon=True).start()

def connect(event):
    if ip_stu == 1: start_network_thread()

# --- Funciones de Botones y Lógica de la GUI ---

# ---- FUNCIONES DE CÁMARA MODIFICADAS (Pulsar de nuevo para centrar) ----
def call_up(event):
    global camera_tilt_state, camera_pan_state
    if camera_tilt_state == 'up':
        send_command('home')
        camera_tilt_state = 'stop'
        camera_pan_state = 'stop'
    else:
        send_command('up')
        camera_tilt_state = 'up'

def call_down(event):
    global camera_tilt_state, camera_pan_state
    if camera_tilt_state == 'down':
        send_command('home')
        camera_tilt_state = 'stop'
        camera_pan_state = 'stop'
    else:
        send_command('down')
        camera_tilt_state = 'down'

def call_lookleft(event):
    global camera_pan_state, camera_tilt_state
    if camera_pan_state == 'left':
        send_command('home')
        camera_pan_state = 'stop'
        camera_tilt_state = 'stop'
    else:
        send_command('lookleft')
        camera_pan_state = 'left'

def call_lookright(event):
    global camera_pan_state, camera_tilt_state
    if camera_pan_state == 'right':
        send_command('home')
        camera_pan_state = 'stop'
        camera_tilt_state = 'stop'
    else:
        send_command('lookright')
        camera_pan_state = 'right'

# ---- FUNCIONES DE DIRECCIÓN SIN CAMBIOS (Ya funcionan como pides) ----
def call_left(event):
    global steering_state
    if steering_state == 'left':
        send_command('TS')
        steering_state = 'center'
    else:
        send_command('left')
        steering_state = 'left'

def call_right(event):
    global steering_state
    if steering_state == 'right':
        send_command('TS')
        steering_state = 'center'
    else:
        send_command('right')
        steering_state = 'right'

# ---- FUNCIONES DE MOVIMIENTO SIN CAMBIOS (MANTENER PULSADO) ----
def call_forward(event): send_command('forward')
def call_backward(event): send_command('backward')
def call_DS(event): send_command('DS')
def call_TS(event): send_command('TS')

# ---- OTRAS FUNCIONES ----
def call_home(event): 
    global camera_pan_state, camera_tilt_state
    send_command('home')
    camera_pan_state = 'stop'
    camera_tilt_state = 'stop'

def call_police(event): send_command('police')
def call_rainbow(event): send_command('rainbow')
def call_sr(event): send_command('sr')
def call_CVrun(event): send_command('CVrun')

def toggle_function(button_widget, start_command, stop_command):
    global function_button_active, motor_controls, servo_controls
    if function_button_active and function_button_active != button_widget:
        function_button_active.config(bg=color_btn)
    if function_button_active == button_widget:
        send_command(stop_command)
        button_widget.config(bg=color_btn)
        function_button_active = None
        for btn in motor_controls + servo_controls: btn.config(state='normal')
    else:
        send_command(start_command)
        button_widget.config(bg='#4CAF50')
        function_button_active = button_widget
        for btn in motor_controls + servo_controls: btn.config(state='disabled')

def call_function_1(event): send_command('scan')
def call_function_2(event):
    # Enciende/apaga lógica en la Pi y sincroniza el picker local
    toggle_function(Btn_function_2, 'findColor', 'stopCV')
    # Tras toggle_function, si el botón activo es el de FindColor, estamos en ON
    if function_button_active == Btn_function_2:
        _hsv_enable(True)
    else:
        _hsv_enable(False)
        
def call_function_3(event): toggle_function(Btn_function_3, 'motionGet', 'stopCV')
def call_function_4(event): toggle_function(Btn_function_4, 'trackLine', 'pauseFunctions')
def call_function_5(event): toggle_function(Btn_function_5, 'automatic', 'pauseFunctions')
def call_function_6(event): send_command('steadyCamera')
def call_function_7(event): pass

def call_Switch_1(event):
    global Switch_1; command = 'Switch_1_on' if Switch_1 == 0 else 'Switch_1_off'
    send_command(command); Switch_1 = 1 - Switch_1
    Btn_Switch_1.config(bg='#4CAF50' if Switch_1 else color_btn)
def call_Switch_2(event):
    global Switch_2; command = 'Switch_2_on' if Switch_2 == 0 else 'Switch_2_off'
    send_command(command); Switch_2 = 1 - Switch_2
    Btn_Switch_2.config(bg='#4CAF50' if Switch_2 else color_btn)
def call_Switch_3(event):
    global Switch_3; command = 'Switch_3_on' if Switch_3 == 0 else 'Switch_3_off'
    send_command(command); Switch_3 = 1 - Switch_3
    Btn_Switch_3.config(bg='#4CAF50' if Switch_3 else color_btn)

def speed_send(event): send_command(f'Speed {var_Speed.get()}')
def R_send(event): send_command(f'wsR {var_R_L.get()}')
def G_send(event): send_command(f'wsG {var_G_L.get()}')
def B_send(event): send_command(f'wsB {var_B_L.get()}')
def pwm0_send(event): send_command(f'pwm0 {var_0.get()}')
def pwm1_send(event): send_command(f'pwm1 {var_1.get()}')
def pwm2_send(event): send_command(f'pwm2 {var_2.get()}')
def call_Save(event): send_command('Save')
def lip1_send(event): send_command(f'lip1 {var_lip1.get()}')
def lip2_send(event): send_command(f'lip2 {var_lip2.get()}')
def err_send(event): send_command(f'err {var_err.get()}')
def call_Render(event): send_command('Render')
def call_CVFL(event): send_command('CVFL')
def call_WB(event): send_command('WBswitch')
def rgb2hsv(r, g, b):
    r,g,b=r/255.0,g/255.0,b/255.0; mx=max(r,g,b); mn=min(r,g,b); df=mx-mn
    if mx==mn: h=0
    elif mx==r: h=(60*((g-b)/df)+360)%360
    elif mx==g: h=(60*((b-r)/df)+120)%360
    elif mx==b: h=(60*((r-g)/df)+240)%360
    s=0 if mx==0 else (df/mx)*100; v=mx*100; h=h/360*255
    return f"{int(h)} {int(s)} {int(v)}"
def call_SET(event): send_command(f'FCSET {rgb2hsv(int(var_R.get()), int(var_G.get()), int(var_B.get()))}')
def EC_send(event): send_command(f'setEC {var_ec.get()}')
def EC_default(event): send_command('defEC')

def update_color_swatch(event=None):
    try:
        r = int(var_R.get()); g = int(var_G.get()); b = int(var_B.get())
        hex_color = f'#{r:02x}{g:02x}{b:02x}'
        canvas_show.config(bg=hex_color)
    except (AttributeError, ValueError): pass

# --- Creación de la Interfaz Gráfica ---
def loop():
    global root, cpu_temp_var, cpu_use_var, ram_use_var, var_Speed, var_R_L, var_G_L, var_B_L, var_0, var_1, var_2, var_lip1, var_lip2, var_err, var_R, var_G, var_B, var_ec, Btn_Switch_1, Btn_Switch_2, Btn_Switch_3, E1, Btn14, l_ip_4, l_ip_5, Btn_function_2, Btn_function_3, Btn_function_4, Btn_function_5, canvas_show
    root = tk.Tk(); root.title('TFG'); root.geometry('665x300'); root.config(bg=color_bg)
    var_Speed=tk.StringVar(); var_Speed.set(100); var_R_L=tk.StringVar(); var_R_L.set(0); var_G_L=tk.StringVar(); var_G_L.set(0); var_B_L=tk.StringVar(); var_B_L.set(0)
    var_R=tk.StringVar(); var_R.set(80); var_G=tk.StringVar(); var_G.set(80); var_B=tk.StringVar(); var_B.set(80); var_0=tk.StringVar(); var_0.set(300)
    var_1=tk.StringVar(); var_1.set(300); var_2=tk.StringVar(); var_2.set(300); var_lip1=tk.StringVar(); var_lip1.set(440); var_lip2=tk.StringVar(); var_lip2.set(380)
    var_err=tk.StringVar(); var_err.set(20); var_ec=tk.StringVar(); var_ec.set(0)
    try:
        logo=tk.PhotoImage(file='logo.png'); tk.Label(root,image=logo,bg=color_bg).place(x=30,y=13)
    except: pass
    cpu_temp_var = tk.StringVar(value="N/A")
    cpu_use_var = tk.StringVar(value="N/A")
    ram_use_var = tk.StringVar(value="N/A")
    motor_buttons(30,105); information_screen(380,15); connent_input(125,15) 
    
    try:
        with open("last_ip.txt") as f:
            last = f.read().strip()
            if last:
                E1.insert(0, last)
    except FileNotFoundError:
        pass
    
    switch_button(30,195); servo_buttons(255,195); scale(30,230,203); function_buttons(550,15)
    #scale_RGB(370,280,172); scale_PWM(370,400,172); ultrasonic_radar(30,290);  scale_FL(30,550,320)
    #scale_FC(30,650,320); scale_ExpCom(30,770,320)
    root.mainloop()

def motor_buttons(x,y):
    global motor_controls
    motor_controls = []
    btn_motor_fwd = tk.Button(root, width=8, text='Forward',fg=color_text,bg=color_btn,relief='ridge'); btn_motor_fwd.place(x=x+70,y=y)
    btn_motor_fwd.bind('<ButtonPress-1>', call_forward); btn_motor_fwd.bind('<ButtonRelease-1>', call_DS)
    root.bind('<KeyPress-w>', call_forward); root.bind('<KeyRelease-w>', call_DS)
    motor_controls.append(btn_motor_fwd)
    btn_motor_bwd = tk.Button(root, width=10, text='Backward',fg=color_text,bg=color_btn,relief='ridge'); btn_motor_bwd.place(x=x+70,y=y+35)
    btn_motor_bwd.bind('<ButtonPress-1>', call_backward); btn_motor_bwd.bind('<ButtonRelease-1>', call_DS)
    root.bind('<KeyPress-s>', call_backward); root.bind('<KeyRelease-s>', call_DS)
    motor_controls.append(btn_motor_bwd)
    btn_motor_left = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge'); btn_motor_left.place(x=x,y=y+35)
    btn_motor_left.bind('<ButtonPress-1>', call_left)
    root.bind('<KeyPress-a>', call_left)
    motor_controls.append(btn_motor_left)
    btn_motor_right = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge'); btn_motor_right.place(x=x+140,y=y+35)
    btn_motor_right.bind('<ButtonPress-1>', call_right)
    root.bind('<KeyPress-d>', call_right)
    motor_controls.append(btn_motor_right)

def servo_buttons(x,y):
    global Btn_SR, Btn_Police, Btn_Rainbow, Btn_3, servo_controls
    servo_controls = []
    btn_servo_up = tk.Button(root, width=8, text='Up',fg=color_text,bg=color_btn,relief='ridge'); btn_servo_up.place(x=x+70,y=y)
    btn_servo_up.bind('<ButtonPress-1>', call_up)
    root.bind('<KeyPress-i>', call_up)
    servo_controls.append(btn_servo_up)
    btn_servo_down = tk.Button(root, width=8, text='Down',fg=color_text,bg=color_btn,relief='ridge'); btn_servo_down.place(x=x+70,y=y+35)
    btn_servo_down.bind('<ButtonPress-1>', call_down)
    root.bind('<KeyPress-k>', call_down)
    servo_controls.append(btn_servo_down)
    btn_servo_left = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge'); btn_servo_left.place(x=x,y=y+35)
    btn_servo_left.bind('<ButtonPress-1>', call_lookleft)
    root.bind('<KeyPress-j>', call_lookleft)
    servo_controls.append(btn_servo_left)
    btn_servo_right = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge'); btn_servo_right.place(x=x+140,y=y+35)
    btn_servo_right.bind('<ButtonPress-1>', call_lookright)
    root.bind('<KeyPress-l>', call_lookright)
    servo_controls.append(btn_servo_right)
    
    btn_center = tk.Button(root, width=8, text='Center',fg=color_text,bg=color_btn,relief='ridge')
    btn_center.place(x=x+140,y=y)
    btn_center.bind('<ButtonPress-1>', call_home) # Llama a la función de centrado

     
    Btn_SR = tk.Button(root, width=8, text='CV Run',fg=color_text,bg=color_btn,relief='ridge'); Btn_SR.place(x=x,y=y)
    Btn_SR.bind('<ButtonPress-1>', call_CVrun); root.bind('<KeyPress-u>', call_CVrun) 
    Btn_Police = tk.Button(root, width=8, text='Police',fg=color_text,bg=color_btn,relief='ridge'); Btn_Police.place(x=x,y=y-55)
    Btn_Police.bind('<ButtonPress-1>', call_police); root.bind('<KeyPress-g>', call_police) 
    Btn_Rainbow = tk.Button(root, width=8, text='Rainbow',fg=color_text,bg=color_btn,relief='ridge'); Btn_Rainbow.place(x=x,y=y-55-35)
    Btn_Rainbow.bind('<ButtonPress-1>', call_rainbow); root.bind('<KeyPress-y>', call_rainbow)
    root.bind('<KeyPress-h>', call_home)
    
def information_screen(x,y):
	global l_ip_4, l_ip_5
    # Etiquetas fijas (los títulos)
	tk.Label(root,width=10,text='CPU Temp:',fg=color_text,bg=color_bg, anchor='w').place(x=x,y=y)
	tk.Label(root,width=10,text='CPU Usage:',fg=color_text,bg=color_bg, anchor='w').place(x=x,y=y+30)
	tk.Label(root,width=10,text='RAM Usage:',fg=color_text,bg=color_bg, anchor='w').place(x=x,y=y+60)

    # Etiquetas dinámicas (los valores) que se actualizan solas
	tk.Label(root,width=8,textvariable=cpu_temp_var,fg=color_text,bg=color_bg, anchor='w').place(x=x+80,y=y)
	tk.Label(root,width=8,textvariable=cpu_use_var,fg=color_text,bg=color_bg, anchor='w').place(x=x+80,y=y+30)
	tk.Label(root,width=8,textvariable=ram_use_var,fg=color_text,bg=color_bg, anchor='w').place(x=x+80,y=y+60)

	l_ip_4=tk.Label(root,width=18,text='Disconnected',fg=color_text,bg='#F44336'); l_ip_4.place(x=x,y=y+95)
	l_ip_5=tk.Label(root,width=18,text='<No IP>',fg=color_text,bg=color_btn); l_ip_5.place(x=x,y=y+130)
def connent_input(x,y):
	global E1, Btn14; E1 = tk.Entry(root,show=None,width=16,bg="#37474F",fg='#eceff1'); E1.place(x=x+5,y=y+25); tk.Label(root,width=10,text='IP Address:',fg=color_text,bg='#000000').place(x=x,y=y)
	Btn14= tk.Button(root, width=8,height=2, text='Connect',fg=color_text,bg=color_btn,relief='ridge'); Btn14.place(x=x+130,y=y); root.bind('<Return>', connect); Btn14.bind('<ButtonPress-1>', connect)
def switch_button(x,y):
	global Btn_Switch_1, Btn_Switch_2, Btn_Switch_3; Btn_Switch_1 = tk.Button(root, width=8, text='Port 1',fg=color_text,bg=color_btn,relief='ridge'); Btn_Switch_2 = tk.Button(root, width=8, text='Port 2',fg=color_text,bg=color_btn,relief='ridge')
	Btn_Switch_3 = tk.Button(root, width=8, text='Port 3',fg=color_text,bg=color_btn,relief='ridge'); Btn_Switch_1.place(x=x,y=y); Btn_Switch_2.place(x=x+70,y=y); Btn_Switch_3.place(x=x+140,y=y)
	Btn_Switch_1.bind('<ButtonPress-1>', call_Switch_1); Btn_Switch_2.bind('<ButtonPress-1>', call_Switch_2); Btn_Switch_3.bind('<ButtonPress-1>', call_Switch_3)
def function_buttons(x,y):
    global Btn_function_2, Btn_function_3, Btn_function_4, Btn_function_5
    Btn_function_1 = tk.Button(root, width=8, text='RadarScan',fg=color_text,bg=color_btn,relief='ridge'); Btn_function_2 = tk.Button(root, width=8, text='FindColor',fg=color_text,bg=color_btn,relief='ridge')
    Btn_function_3 = tk.Button(root, width=8, text='MotionGet',fg=color_text,bg=color_btn,relief='ridge'); Btn_function_4 = tk.Button(root, width=8, text='LineTrack',fg=color_text,bg=color_btn,relief='ridge')
    Btn_function_5 = tk.Button(root, width=8, text='Automatic',fg=color_text,bg=color_btn,relief='ridge'); Btn_function_6 = tk.Button(root, width=8, text='SteadyCam',fg=color_text,bg=color_btn,relief='ridge')
    Btn_function_7 = tk.Button(root, width=8, text='Instruction',fg=color_text,bg=color_btn,relief='ridge'); Btn_function_1.place(x=x,y=y); Btn_function_2.place(x=x,y=y+35); Btn_function_3.place(x=x,y=y+70)
    Btn_function_4.place(x=x,y=y+105); Btn_function_5.place(x=x,y=y+140); Btn_function_7.place(x=x,y=y+215); Btn_function_1.bind('<ButtonPress-1>', call_function_1)
    Btn_function_2.bind('<ButtonPress-1>', call_function_2); Btn_function_3.bind('<ButtonPress-1>', call_function_3); Btn_function_4.bind('<ButtonPress-1>', call_function_4)
    Btn_function_5.bind('<ButtonPress-1>', call_function_5); Btn_function_7.bind('<ButtonPress-1>', call_function_7)
def scale(x,y,w):
	tk.Scale(root,label=None,from_=60,to=100,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=10,variable=var_Speed,troughcolor='#448AFF',command=speed_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)

def scale_RGB(x,y,w):
	tk.Scale(root,label=None,from_=0,to=255,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_R_L,troughcolor='#F44336',command=R_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)
	tk.Scale(root,label=None,from_=0,to=255,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_G_L,troughcolor='#4CAF50',command=G_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+30)
	tk.Scale(root,label=None,from_=0,to=255,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_B_L,troughcolor='#2979FF',command=B_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+60)
"""
def scale_PWM(x,y,w):
	tk.Scale(root,label=None,from_=200,to=400,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_0,troughcolor='#212121',command=pwm0_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)
	tk.Scale(root,label=None,from_=200,to=400,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_1,troughcolor='#212121',command=pwm1_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+30)
	tk.Scale(root,label=None,from_=200,to=400,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_2,troughcolor='#212121',command=pwm2_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+60)
	Btn_Save = tk.Button(root, width=23, text='Save as Default',fg=color_text,bg='#212121',relief='ridge'); Btn_Save.place(x=x+1,y=y+110); Btn_Save.bind('<ButtonPress-1>', call_Save)
def ultrasonic_radar(x,y):
    global can_scan
    can_scan = tk.Canvas(root,bg=color_can,height=250,width=320,highlightthickness=0); can_scan.place(x=x,y=y); can_scan.create_line(0,62,320,62,fill='darkgray'); can_scan.create_line(0,124,320,124,fill='darkgray')
    can_scan.create_line(0,186,320,186,fill='darkgray'); can_scan.create_line(160,0,160,250,fill='darkgray'); can_scan.create_line(80,0,80,250,fill='darkgray'); can_scan.create_line(240,0,240,250,fill='darkgray')
    can_scan.create_text((27,178),text='%sm'%round((2/4),2),fill='#aeea00'); can_scan.create_text((27,116),text='%sm'%round((2/2),2),fill='#aeea00'); can_scan.create_text((27,54),text='%sm'%round((2*0.75),2),fill='#aeea00')

def draw_radar_scan(scan_data):
    if not 'can_scan' in globals(): return
    can_scan.delete("scan_point")
    center_x = 160; center_y = 250; max_dist_cm = 200.0
    for item in scan_data:
        dist_m, angle_deg = item
        dist_cm = dist_m * 100
        if dist_cm <= 0 or dist_cm > max_dist_cm: continue
        angle_rad = math.radians(angle_deg)
        pixel_dist = (dist_cm / max_dist_cm) * 250
        x = center_x - pixel_dist * math.cos(angle_rad)
        y = center_y - pixel_dist * math.sin(angle_rad)
        can_scan.create_oval(x-3, y-3, x+3, y+3, fill=target_color, outline=target_color, tags="scan_point")

def scale_FL(x,y,w):
	global Btn_CVFL; tk.Scale(root,label=None,from_=0,to=480,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_lip1,troughcolor='#212121',command=lip1_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)
	tk.Scale(root,label=None,from_=0,to=480,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_lip2,troughcolor='#212121',command=lip2_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+30)
	tk.Scale(root,label=None,from_=0,to=200,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_err,troughcolor='#212121',command=err_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+60)
	Btn_Render = tk.Button(root, width=10, text='Render',fg=color_text,bg='#212121',relief='ridge'); Btn_Render.place(x=x+w+111,y=y+20); Btn_Render.bind('<ButtonPress-1>', call_Render)
	Btn_CVFL = tk.Button(root, width=10, text='CV FL',fg=color_text,bg='#212121',relief='ridge'); Btn_CVFL.place(x=x+w+21,y=y+20); Btn_CVFL.bind('<ButtonPress-1>', call_CVFL)
	Btn_WB = tk.Button(root, width=23, text='LineColorSwitch',fg=color_text,bg='#212121',relief='ridge'); Btn_WB.place(x=x+w+21,y=y+60); Btn_WB.bind('<ButtonPress-1>', call_WB)

def scale_FC(x,y,w):
	global canvas_show
	tk.Scale(root,label=None,from_=0,to=255,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_R,troughcolor='#FF1744',command=update_color_swatch,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)
	tk.Scale(root,label=None,from_=0,to=255,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_G,troughcolor='#00E676',command=update_color_swatch,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+30)
	tk.Scale(root,label=None,from_=0,to=255,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_B,troughcolor='#2979FF',command=update_color_swatch,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y+60)
	canvas_show=tk.Canvas(root,bg=f'#{int(var_R.get()):02x}{int(var_G.get()):02x}{int(var_B.get()):02x}',height=35,width=170,highlightthickness=0); canvas_show.place(x=w+x+21,y=y+15)
	Btn_WB = tk.Button(root, width=23, text='Color Set',fg=color_text,bg='#212121',relief='ridge'); Btn_WB.place(x=x+w+21,y=y+60); Btn_WB.bind('<ButtonPress-1>', call_SET)
def scale_ExpCom(x,y,w):
	tk.Scale(root,label='Exposure Compensation Level', from_=-25,to=25,orient=tk.HORIZONTAL,length=w,showvalue=1,tickinterval=None,resolution=1,variable=var_ec,troughcolor='#212121',command=EC_send,fg=color_text,bg=color_bg,highlightthickness=0).place(x=x,y=y)
	Btn_dEC = tk.Button(root, width=23,height=2, text='Set Default Exposure\nCompensation Level',fg=color_text,bg='#212121',relief='ridge'); Btn_dEC.place(x=x+w+21,y=y+3); Btn_dEC.bind('<ButtonPress-1>', EC_default)
"""
if __name__ == '__main__':
    loop()