#!/usr/bin/env python3
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

# Nombre del archivo: webServer.py
# Descripción: Servidor Principal del Sistema.
#               Integra múltiples servicios Concurrentes:
#               1. Servidor WebSocket (Puerto 8888): Control remoto en tiempo real y telemetría.
#               2. Servidor Flask (Puerto 5000): Streaming de video HTTP.
#               3. Lógica de Hardware: Coordinación de motores, servos y sensores.

import os
import asyncio
import websockets
import json
import socket
import threading

# Módulos del robot
from camera_opencv import Camera  # Sistema de cámara
import move                       # Control de motores DC
import RPIservo                   # Control de servos (PCA9685)
import functions                  # Lógica autónoma (PID, Line Following)
import robotLight                 # Control de iluminación RGB
import app                        # Instancia de aplicación Flask

# =================== Estado global del Sistema ===================
speed_set = 70             # Velocidad por defecto (0-100)
direction_command = 'no'   # Estado actual de dirección
turn_command = 'no'        # Estado actual de giro

# Identificadores de canales de Servo
SERVO_TILT = 0     # Servo de inclinación vertical de cámara
SERVO_PAN  = 1     # Servo de paneo horizontal de cámara
SERVO_STEERING = 2 # Servo de dirección del vehículo

# Instancias Globales de Controladores
RL = None   # Controlador de Luces
fuc = None  # Controlador de Funciones Autónomas

# =================== Utilidades de Sistema (Optimizado) ===================
# Variables estáticas para cálculo diferencial de uso de CPU
_prev_cpu_total = None
_prev_cpu_idle  = None

def _read_cpu_times():
    """
    Lee las estadísticas crudas del kernel desde /proc/stat.
    Optimización: Evita usar comandos externos como 'top' que son lentos y pesados.
    
    Retorna:
    - total: Tiempo total de CPU acumulado.
    - idle: Tiempo ocioso acumulado (idle + iowait).
    """
    try:
        with open("/proc/stat", "r") as f:
            line = f.readline()
            if not line.startswith("cpu "): return None, None
            parts = list(map(int, line.split()[1:11]))
            # idle incluye idle normal + iowait (espera de disco)
            idle_all = parts[3] + parts[4] 
            total = sum(parts)
            return total, idle_all
    except: return None, None

def get_cpu_usage():
    """
    Calcula el porcentaje de uso de CPU en tiempo real.
    Compara los contadores de ciclos entre dos llamadas consecutivas.
    """
    global _prev_cpu_total, _prev_cpu_idle
    t_now, i_now = _read_cpu_times()
    
    if t_now is None: return "0"
    
    # Primera lectura: no hay referencia anterior
    if _prev_cpu_total is None:
        _prev_cpu_total, _prev_cpu_idle = t_now, i_now
        return "0" 
    
    delta_total = t_now - _prev_cpu_total
    delta_idle  = i_now - _prev_cpu_idle
    
    # Actualizar estado previo
    _prev_cpu_total, _prev_cpu_idle = t_now, i_now
    
    if delta_total == 0: return "0"
    usage = 100.0 * (1.0 - (delta_idle / delta_total))
    return f"{usage:.1f}"

def get_cpu_temp():
    """Obtiene la temperatura del SoC leyendo la zona térmica del sistema."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(float(f.read()) / 1000, 2)
    except: return 0.0

def get_ram_usage():
    """Calcula el porcentaje de memoria RAM utilizada leyendo /proc/meminfo."""
    try:
        mem = {}
        with open("/proc/meminfo", "r") as f:
            for line in f:
                parts = line.split(':')
                if len(parts) == 2:
                    mem[parts[0].strip()] = int(parts[1].split()[0])
        
        total = mem.get('MemTotal', 1)
        avail = mem.get('MemAvailable', mem.get('MemFree', 0))
        used_percent = 100 * (1 - (avail / total))
        return f"{used_percent:.1f}"
    except: return "0"

def wifi_check():
    """
    Verificación de conectividad de red al inicio.
    - Si hay internet (ping a 1.1.1.1): pone luces VERDES.
    - Si no hay red: intenta crear un punto de acceso (AP) y pone luces AZULES.
    """
    try:
        # Intenta conectar a DNS de Cloudflare (sin enviar datos, solo handshake TCP)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        s.connect(("1.1.1.1", 80))
        s.close()
        print("[Network] Wi-Fi conectado a infraestructura.")
        if RL: RL.front_color('green')
    except Exception:
        print("[Network] Sin conexión. Intentando levantar Hotspot (AP)...")
        if RL: RL.front_color('blue')
        try:
            # Script de sistema para modo AP (requiere configuración previa en SO)
            os.system("sudo create_ap wlan0 eth0 Adeept_Robot &")
        except: pass

def servoPosInit():
    """Restablece los servos a su posición segura (Home)."""
    RPIservo.move(SERVO_STEERING, 88.5) # Centro dirección
    RPIservo.move(SERVO_TILT, 40)       # Cabeza levemente abajo
    RPIservo.move(SERVO_PAN, 85)        # Cabeza al frente

# =================== Lógica de Control del Robot ===================
def robotCtrl(cmd, ws_obj):
    """
    Intérprete de comandos de bajo nivel.
    Mapea instrucciones textuales a acciones de hardware.
    
    Args:
        cmd (str): Comando recibido (ej. 'forward', 'left').
        ws_obj: Objeto WebSocket (no usado directamente aquí, pero disponible para feedback).
    """
    global direction_command, turn_command, speed_set

    # --- Control de Tracción (Motores DC) ---
    if cmd == 'forward':
        move.forward(speed_set)
        if RL: RL.front_color('blue')
    elif cmd == 'backward':
        move.backward(speed_set)
        if RL: RL.front_color('red') # Luz de freno/reversa
    elif cmd == 'DS': # Drive Stop: Detención inmediata
        move.motorStop()
        if RL: RL.front_all_off()
    elif cmd == 'TS': # Turn Stop: Centrar dirección
        RPIservo.move(SERVO_STEERING, 88.5)
        
    # --- Control de Dirección (Servo Eje Delantero) ---
    elif cmd == 'left':
        RPIservo.move(SERVO_STEERING, 120) # Límite izquierdo
        if RL: RL.front_turn_left()
    elif cmd == 'right':
        RPIservo.move(SERVO_STEERING, 50)  # Límite derecho
        if RL: RL.front_turn_right()
        
    # --- Control de Cámara (Servos Pan/Tilt) ---
    elif cmd == 'up':
        RPIservo.move(SERVO_TILT, 110) # Mirar arriba
    elif cmd == 'down':
        RPIservo.move(SERVO_TILT, 65)  # Mirar abajo
    elif cmd == 'home':
        servoPosInit()
    elif cmd == 'lookleft':
        RPIservo.move(SERVO_PAN, 120)
    elif cmd == 'lookright':
        RPIservo.move(SERVO_PAN, 50)

# =================== Lógica del Servidor WebSocket ===================
async def recv_msg(websocket, path):
    """
    Bucle principal de comunicación WebSocket por cliente.
    Recibe, parsea y ejecuta comandos en tiempo real.
    """
    global speed_set
    
    while True:
        try:
            data = await websocket.recv()

            # -----------------------------------------------
            # 1. Comandos de Movimiento Directo
            # -----------------------------------------------
            if data in ['forward', 'backward', 'DS', 'TS', 'left', 'right',
                        'lookleft', 'lookright', 'up', 'down', 'home']:
                robotCtrl(data, websocket)

            # -----------------------------------------------
            # 2. Configuración de Velocidad
            # -----------------------------------------------
            elif data.startswith('Speed'):
                try:
                    val = int(data.split()[1])
                    speed_set = move.speed_set(val) # Actualiza valor y retorna el limitado
                except: pass

            # -----------------------------------------------
            # 3. Modos Autónomos (Computer Vision)
            # -----------------------------------------------
            elif data == 'trackLine':
                # Activa seguimiento de línea
                if RL: RL.front_color('white')
                fuc.modeSet('trackLine')
                
            elif data == 'findColor':
                # Activa búsqueda de color (Feature legacy)
                if RL: RL.front_color('yellow')
                Camera().modeselect('findColor')

            elif data == 'stopCV' or data == 'pauseFunctions':
                # Detiene cualquier modo autónomo
                if RL: RL.front_color('black') # Luces apagadas
                fuc.pause()
                Camera().modeselect('none')

            # -----------------------------------------------
            # 4. Telemetría de Sistema
            # -----------------------------------------------
            elif data == 'get_info':
                # Responde con un JSON conteniendo estado de salud del sistema
                info = json.dumps({
                    'title': 'info_update',
                    'data': {
                        'CPU_Temp': get_cpu_temp(),   # ºC
                        'CPU_Usage': get_cpu_usage(), # %
                        'RAM_Usage': get_ram_usage()  # %
                    }
                })
                await websocket.send(info)

            # -----------------------------------------------
            # 5. Mantenimiento de Conexión
            # -----------------------------------------------
            elif data == 'ping':
                pass # Keep-alive

        except websockets.exceptions.ConnectionClosed:
            print("[WebSocket] Cliente desconectado (ConnectionClosed)")
            move.motorStop() # Seguridad: Detener robot si se pierde control
            break
        except Exception as e:
            print(f"[WebSocket] Error de protocolo: {e}")
            break

async def main_logic(websocket, path):
    """
    Handshake inicial de conexión WebSocket.
    Implementa autenticación básica antes de permitir control.
    """
    try:
        # Espera credenciales como primer mensaje
        auth = await websocket.recv()
        # Credenciales harcodeadas (Por defecto de fábrica)
        if auth.strip() == "admin:123456":
            await websocket.send("congratulation")
            await recv_msg(websocket, path) # Transferir a bucle de comandos
        else:
            await websocket.send("sorry")
    except: pass

# =================== Punto de Entrada Principal ===================
if __name__ == '__main__':
    print("[Init] Inicializando sistema Adeept PiCar-B...")
    
    # 1. Inicialización de Hardware Base (Motores/Servos)
    move.setup()
    RPIservo.init() if hasattr(RPIservo, 'init') else None 
    servoPosInit()

    # 2. Inicialización de Subsistema de Iluminación
    try:
        RL = robotLight.RobotLight()
    except Exception as e:
        print(f"[Init] Error iniciando luces: {e}")
        RL = None

    # 3. Inicialización de Controlador Autónomo (Hilo independiente)
    fuc = functions.Functions()
    fuc.start()

    # 4. Inicialización de Cámara (Singleton)
    # Nota: Instanciar Camera() configura el hardware, los hilos de captura se inician internamente.
    try:
        cam = Camera()
        print("[Init] Cámara iniciada y lista.")
    except Exception as e:
        print(f"[Init] Error crítico en cámara: {e}")

    # 5. Servidor de Streaming de Video (Flask)
    # Se ejecuta en un hilo Demonio para no bloquear el Event Loop de asyncio.
    try:
        flask_thread = threading.Thread(target=lambda: app.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
        flask_thread.daemon = True
        flask_thread.start()
        print("[Init] Servidor Video (Flask) escuchando en puerto 5000")
    except Exception as e:
        print(f"[Init] Error iniciando Flask: {e}")

    # 6. Comprobación de Red
    wifi_check()

    # 7. Servidor de Control (WebSocket)
    print("[Init] Servidor Control (WebSocket) iniciando en puerto 8888...")
    
    # Configuración del Event Loop de Asyncio
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(main_logic, '0.0.0.0', 8888)

    try:
        loop.run_until_complete(start_server)
        print("[System] Sistema en línea. Esperando conexiones.")
        loop.run_forever()
    except KeyboardInterrupt:
        print("\n[System] Apagando sistema ordenadamente...")
    finally:
        # Limpieza de recursos al salir (CTRL+C)
        move.motorStop()
        move.destroy()
        if RL: RL.cleanup()