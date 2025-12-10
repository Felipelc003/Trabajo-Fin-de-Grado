#!/usr/bin/env python3
# File name   : webServer.py
# Description : WebSocket server para controlar PiCar-B (1 motor + servo dirección)
# Autor: Adeept | Adaptado para TFG (RobotLight API actual + move.py canal A)

import os
import sys
import time
import threading
import asyncio
import websockets
import json
import socket
from camera_opencv import Camera
import move
import RPIservo
import functions
import robotLight
import switch
import app

# =================== Estado global ===================
speed_set = 70
direction_command = 'no'
turn_command = 'no'

# Servo IDs (según tu PCA9685)
SERVO_TILT = 0
SERVO_PAN = 1
SERVO_STEERING = 2

# =================== Luces ===================
try:
    RL = robotLight.RobotLight()
    RL.start()  # hilo de efectos (breath, police, rainbow...)
except Exception as e:
    RL = None
    print(f"[robotLight] No disponible: {e}")

# =================== Hilo de funciones (radar, automático, sigue-líneas) ===================
fuc = functions.Functions()
fuc.start()

# === Helper para compatibilidad de modos de cámara (nuevo/antiguo) ===
def camera_mode(mode: str):
    """
    Puente de compatibilidad:
      - Si la nueva cámara tiene enable_line_black(), úsala.
      - Si tu cámara antigua tiene modeselect(), úsala como fallback.
    """
    try:
        cam = Camera.get_instance()
        # nueva API
        if hasattr(cam, "enable_line_black"):
            if mode in ("lineBlack", "trackLine", "automatic"):
                cam.enable_line_black(True)
            elif mode in ("none", "pause", "stop"):
                cam.enable_line_black(False)
            else:
                # otros modos que lleve tu app (findColor/watchDog) viven en app.flask_app
                pass
        # legacy (por si sigue existiendo en alguna rama)
        elif hasattr(cam, "modeselect"):
            cam.modeselect(mode)
    except Exception as e:
        print(f"[camera_mode] No se pudo cambiar modo '{mode}': {e}")

# =================== Utilidades ===================
def servoPosInit():
    """Centra dirección, pan y tilt."""
    RPIservo.move(SERVO_STEERING, 88.5)
    RPIservo.move(SERVO_TILT, 40)
    RPIservo.move(SERVO_PAN, 80)

def wifi_check():
    """
    Señaliza estado de red con RobotLight:
    - Conectado: frontal verde + traseros verde
    - AP: frontal azul + traseros azul
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.5)
        s.connect(("1.1.1.1", 80))
        s.close()
        print("Wi-Fi conectado.")
        if RL:
            RL.front_color('green')           # delanteros en verde
            RL.rear_set_color(0, 255, 255)      # traseros verde
            # efecto suave opcional en traseros:
            RL.breath(0.0, 1.0, 0.0)          # verde respirando
    except Exception:
        print("No hay Wi-Fi, creando AP...")
        if RL:
            RL.front_color('blue')            # delanteros en azul
            RL.rear_set_color(0, 0, 255)      # traseros azul
            RL.breath(0.0, 0.0, 1.0)          # azul respirando
        os.system("sudo create_ap wlan0 eth0 Adeept_Robot &")

_prev_cpu_total = None
_prev_cpu_idle  = None

def _read_cpu_times():
    """
    Lee /proc/stat y devuelve (total, idle) como enteros.
    total = suma de todos los campos
    idle  = idle + iowait
    """
    with open("/proc/stat", "r") as f:
        first = f.readline()
    if not first.startswith("cpu "):
        return None, None
    parts = first.split()[1:]
    vals = list(map(int, parts[:10]))  # user nice system idle iowait irq softirq steal guest guest_nice
    user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice = vals + [0]*(10-len(vals))
    idle_all = idle + iowait
    total = sum(vals)
    return total, idle_all

def read_cpu_usage_percent():
    """
    Devuelve el % de CPU usado (0-100) calculado con delta entre lecturas.
    En la primera llamada hace una doble lectura rápida.
    """
    global _prev_cpu_total, _prev_cpu_idle
    t1, i1 = _read_cpu_times()
    if t1 is None:
        return 0.0

    # Primera vez: esperar un instante y medir de nuevo
    if _prev_cpu_total is None or _prev_cpu_idle is None:
        time.sleep(0.2)
        t2, i2 = _read_cpu_times()
        if t2 is None:
            return 0.0
        _prev_cpu_total, _prev_cpu_idle = t2, i2
        dt, di = (t2 - t1), (i2 - i1)
    else:
        dt, di = (t1 - _prev_cpu_total), (i1 - _prev_cpu_idle)
        _prev_cpu_total, _prev_cpu_idle = t1, i1

    if dt <= 0:
        return 0.0
    # uso = (tiempo activo / total) * 100 = (dt - di) / dt
    usage = (dt - di) * 100.0 / dt
    return max(0.0, min(100.0, round(usage, 1)))

def read_cpu_temp_c():
    """
    Devuelve temperatura CPU en ºC (float). Usa thermal_zone0 y fallback vcgencmd.
    """
    # thermal_zone0 (típico en RPi OS)
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            raw = f.readline().strip()
        val = float(raw) / 1000.0
        return round(val, 2)
    except Exception:
        pass

    # vcgencmd (si firmware/paquete disponible)
    try:
        out = subprocess.check_output(["/usr/bin/vcgencmd", "measure_temp"], text=True).strip()
        # formato: temp=48.0'C
        if "temp=" in out:
            s = out.split("temp=")[1]
            s = s.split("'")[0]
            return round(float(s), 2)
    except Exception:
        pass

    return 0.0

def read_ram_usage_mb_and_percent():
    """
    Devuelve (used_mb, percent) usando MemAvailable para cálculo real de uso.
    """
    meminfo = {}
    with open("/proc/meminfo", "r") as f:
        for line in f:
            key, val = line.split(":", 1)
            meminfo[key.strip()] = val.strip()

    def _kb(field):
        v = meminfo.get(field, "0 kB").split()[0]
        try:
            return int(v)
        except Exception:
            return 0

    total_kb = _kb("MemTotal")
    avail_kb = _kb("MemAvailable")  # mejor indicador de memoria disponible real
    used_kb = max(0, total_kb - avail_kb)

    used_mb = used_kb // 1024
    percent = 0.0 if total_kb == 0 else (used_kb * 100.0 / total_kb)
    return used_mb, round(percent, 1)

def robotCtrl(command_input, response):
    """Traduce comandos básicos a acciones de motor/servo/luces."""
    global direction_command, turn_command, speed_set

    if command_input == 'forward':
        direction_command = 'forward'
        move.forward(speed_set)
        if RL: RL.front_color('blue')

    elif command_input == 'backward':
        direction_command = 'backward'
        move.backward(speed_set)
        if RL: RL.front_color('red')  # rojo al retroceder

    elif 'DS' in command_input:  # Stop
        direction_command = 'no'
        move.stop()
        if RL: RL.front_all_off()

    elif 'TS' in command_input:  # Straight steering
        turn_command = 'no'
        RPIservo.move(SERVO_STEERING, 88.5)
        if RL: RL.front_all_off()

    elif command_input == 'left':
        turn_command = 'left'
        RPIservo.move(SERVO_STEERING, 120)
        if RL: RL.front_turn_left()  # en tu clase ahora pone blanco

    elif command_input == 'right':
        turn_command = 'right'
        RPIservo.move(SERVO_STEERING, 50)
        if RL: RL.front_turn_right()  # blanco

    elif command_input == 'up':
        RPIservo.move(SERVO_TILT, 110)

    elif command_input == 'down':
        RPIservo.move(SERVO_TILT, 65)

    elif command_input == 'home':
        RPIservo.move(SERVO_TILT, 63)

# =================== WebSocket server ===================
async def recv_msg(websocket, path):
    """Bucle principal de mensajes después del login."""
    global speed_set
    loop = asyncio.get_event_loop()

    while True:
        try:
            data = await websocket.recv()

            # ---- Comandos básicos de texto ----
            if data in ['forward', 'backward', 'DS', 'TS', 'left', 'right',
                        'lookleft', 'lookright', 'up', 'down', 'home']:
                robotCtrl(data, websocket)

            # ---- Velocidad: "Speed <valor>" ----
            elif data.startswith('Speed'):
                try:
                    _, val = data.split()
                    speed_set = move.speed_set(int(val))
                except Exception:
                    pass

            # ---- Radar (scan único) ----
            elif data == 'scan':
                scan_data = fuc.radarScan()
                await websocket.send(json.dumps({
                    'title': 'scanResult',
                    'data': scan_data
                }))

            # ---- QR (scan rq)
            elif data == 'scanQR':
                try:
                    Camera.get_instance().modeselect('scanQR')
                except Exception as e:
                    print(f"[WS] No se pudo activar scanQR: {e}")

            # ---- Modo seguimiento de línea ----
            elif data == 'trackLine':
                if RL: RL.front_color('white')
                fuc.modeSet('trackLine')
                camera_mode('trackline')

            # ---- Modo automático (evitación obst.) ----
            elif data == 'automatic':   # o 'command' según tu variable
                print("[WS] Botón 'automatic' → activar trackLine (lineBlack).")
                try:
                    fuc.modeSet('trackLine')   # usa tu mismo mecanismo de funciones
                except Exception as e:
                    print(f"[WS] No pude lanzar trackLine via fuc.modeSet: {e}")
                camera_mode('lineBlack')

            elif data == 'pauseFunctions':
                if RL: RL.front_color('black')
                fuc.pause()
                camera_mode('none')

            # ---- Telemetría sistema ----
            elif data == 'get_info':
                cpu_temp_raw = os.popen("cat /sys/class/thermal/thermal_zone0/temp").readline()
                cpu_temp = round(float(cpu_temp_raw) / 1000, 2)
                cpu_usage = os.popen("top -n1 | awk '/Cpu/ {print $2}'").readline().strip()
                ram_usage = os.popen("free -m | awk 'NR==2{print $3}'").readline().strip()
                await websocket.send(json.dumps({
                    'title': 'info_update',
                    'data': {
                        'CPU_Temp': cpu_temp,
                        'CPU_Usage': cpu_usage,
                        'RAM_Usage': ram_usage
                    }
                }))

            # ---- Visión por computadora ----
            elif data == 'findColor':
                if RL: RL.front_color('black')
                app.flask_app.modeselect('findColor')

            elif data == 'motionGet':
                app.flask_app.modeselect('watchDog')

            elif data == 'stopCV':
                app.flask_app.modeselect('none')

            elif data.startswith('FCSET'):
                try:
                    _, h, s, v = data.split()
                    app.flask_app.colorFindSet(int(h), int(s), int(v))
                except Exception:
                    pass

            # ---- Modos de luces especiales ----
            elif data == 'police':
                if RL: RL.police()

            elif data == 'rainbow':
                if RL: RL.rainbow()

        except websockets.exceptions.ConnectionClosed:
            print("Cliente desconectado")
            move.stop()
            break
        except Exception as e:
            print(f"[WS] Error procesando comando: {e}")

async def main_logic(websocket, path):
    """Login simple por primer mensaje."""
    try:
        auth = await websocket.recv()
        if auth.strip() == "admin:123456":
            await websocket.send("congratulation")
            await recv_msg(websocket, path)
        else:
            await websocket.send("sorry")
    except websockets.exceptions.ConnectionClosed:
        pass

# =================== Main ===================
if __name__ == '__main__':
    # GPIO de switches (si los usas)
    switch.switchSetup()
    switch.set_all_switch_off()

    # Motor único en canal A
    move.setup()

    # Servos a home
    servoPosInit()

    # Arranque de servidor de cámara/FPV (tu app debe exponer flask_app)
    # Si tu implementación requiere otra inicialización, ajústalo aquí.
    try:
        app.flask_app.startthread()
    except Exception:
        try:
            flask_app = app.webapp()
            flask_app.startthread()
            app.flask_app = flask_app
        except Exception as e:
            print(f"[camera] Aviso: no se pudo iniciar el hilo de cámara: {e}")

    Camera.get_instance().start_background_feed()


    # Señalización de red (usa RobotLight API actual)
    wifi_check()

    # WebSocket en 0.0.0.0:8888
    start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
    asyncio.get_event_loop().run_until_complete(start_server)

    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        move.stop()
        move.destroy()
        RPIservo.cleanup()
        if RL: RL.cleanup()
