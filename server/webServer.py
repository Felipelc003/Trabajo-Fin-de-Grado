#!/usr/bin/python3
# File name   : webServer.py
# Description : Servidor principal para PiCar-B (Versión Final Definitiva)
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

import time, threading, move, info, RPIservo, functions, robotLight, socket, asyncio, websockets, json, app

SERVO_TILT, SERVO_PAN, SERVO_STEERING = 0, 1, 2
speed_set = 100
fuc = functions.Functions()
fuc.start()
RL = None
flask_app = None

def servoPosInit():
    print("Centrando servos...")
    RPIservo.move(SERVO_STEERING, 90); RPIservo.move(SERVO_PAN, 90); RPIservo.move(SERVO_TILT, 90)

def robotCtrl(command):
    # ... (sin cambios)
    if RL:
        if 'forward' == command: RL.front_color('blue')
        elif 'left' == command: RL.front_turn_left()
        elif 'right' == command: RL.front_turn_right()
        elif 'DS' in command or 'TS' in  command: RL.front_color('cian')
        elif 'backward' == command: RL.front_color('yellow')
    
    if 'forward' == command: move.motor(1, 0, speed_set);
    elif 'backward' == command: move.motor(1, 1, speed_set);
    elif 'DS' in command: move.motorStop()
    elif 'left' == command: RPIservo.move(SERVO_STEERING, 135)
    elif 'right' == command: RPIservo.move(SERVO_STEERING, 45)
    elif 'TS' in command: RPIservo.move(SERVO_STEERING, 90)
    elif 'lookleft' == command: RPIservo.move(SERVO_PAN, 135)
    elif 'lookright' == command: RPIservo.move(SERVO_PAN, 45)
    elif 'up' == command: RPIservo.move(SERVO_TILT, 135)
    elif 'down' == command: RPIservo.move(SERVO_TILT, 45)
    elif 'home' == command: servoPosInit()

def wifi_check():
    # ... (sin cambios)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("1.1.1.1", 80)); s.close()
        print("CONEXION WIFI OK")
        if RL: RL.front_color('green')
    except:
        print("CONEXION WIFI FALLIDA. Creando punto de acceso (Hotspot)...")
        ap = threading.Thread(target=lambda: os.system("sudo create_ap wlan0 eth0 Adeept_Robot 12345678"))
        ap.daemon = True
        ap.start()
        if RL: RL.front_color('blue')

async def recv_msg(websocket):
    global speed_set
    while True:
        try:
            raw_data = await websocket.recv()
            parts = raw_data.split(); command = parts[0]
            value = ' '.join(parts[1:]) if len(parts) > 1 else None
            
            if command in ['forward','backward','DS','left','right','TS','lookleft','lookright','up','down','home']:
                robotCtrl(command)
            
            elif command == 'scan':
                print("📡 Iniciando escaneo de radar...")
                scan_data = fuc.radarScan()
                response = {'title': 'scanResult', 'data': scan_data}
                await websocket.send(json.dumps(response))

            elif command == 'trackLine':
                print("Función 'Seguimiento de Línea' activada.")
                servoPosInit()
                fuc.trackLine()

            elif command == 'automatic':
                print("🤖 Activando modo 'Automático' con radar en vivo.")
                fuc.automatic(websocket, asyncio.get_event_loop())

            elif command == 'pauseFunctions':
                print("Pausando todas las funciones activas.")
                fuc.pause()

            elif command == 'findColor':
                print("🎥 Activando modo 'Buscar Color' en el stream de vídeo.")
                if flask_app:
                    flask_app.modeselect('findColor')

            elif command == 'motionGet':
                print("🎥 Activando modo 'Detección de Movimiento'.")
                if flask_app:
                    flask_app.modeselect('watchDog')

            elif command == 'stopCV':
                print("🎥 Deteniendo todos los modos de Visión Artificial.")
                if flask_app:
                    flask_app.modeselect('none')

            # ---- NUEVO BLOQUE PARA OBTENER INFORMACIÓN ----
            elif command == 'get_info':
                # No imprimimos nada para no llenar la consola del servidor
                try:
                    info_data = {
                        'cpu_temp': info.get_cpu_tempfunc(),
                        'cpu_use': info.get_cpu_use(),
                        'ram_use': info.get_ram_info()[1] # get_ram_info() devuelve (total, used), queremos el segundo valor
                    }
                    response = {'title': 'info_update', 'data': info_data}
                    await websocket.send(json.dumps(response))
                except Exception as e:
                    print(f"Error al obtener info del sistema: {e}")
            # ---- FIN DEL NUEVO BLOQUE ----

            elif command in ['police','rainbow'] and RL:
                getattr(RL, command)()
            
            elif command == 'Speed' and value:
                speed_set = int(value)
                move.speed_set(speed_set)
            
            elif command == 'FCSET' and value:
                print(f"🎨 Recibidos nuevos valores HSV: {value}")
                try:
                    h, s, v = map(int, value.split())
                    if flask_app:
                        flask_app.colorFindSet(h, s, v)
                except Exception as e:
                    print(f"Error al procesar valores HSV: {e}")

        except websockets.exceptions.ConnectionClosed:
            move.motorStop(); 
            if RL: RL.front_all_off(); RL.breath(0.3, 0.3, 1.0)
            break
        except Exception as e:
            print(f"Error procesando comando: {e}")

async def main_logic(websocket, path):
    if await websocket.recv() == "admin:123456":
        await websocket.send("congratulation"); await recv_msg(websocket)
    else:
        await websocket.send("sorry")

async def main():
    async with websockets.serve(main_logic, "0.0.0.0", 8888):
        await asyncio.Future()

if __name__ == '__main__':
    try:
        RL = robotLight.RobotLight()
        RL.start()
    except Exception as e: 
        print(f"ADVERTENCIA: No se pudo instanciar el controlador de luces: {e}")
        RL = None
    
    servoPosInit()
    flask_app = app.webapp()
    fuc.camera = flask_app # Damos al hilo de funciones acceso a la cámara
    flask_app.startthread()
    wifi_check()
    
    try:
        print("Servidor Websocket esperando en 0.0.0.0:8888..."); asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServidor detenido.")
    finally:
        print("Limpiando recursos..."); move.destroy(); RPIservo.cleanup()
        if RL: RL.cleanup()
