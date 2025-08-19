#!/usr/bin/python3
# ============================================================================
# Script de Instalación Completa para Adeept PiCar-B
# Actualizado para Raspberry Pi OS Bookworm (64-bit) con librerías modernas
# Autor: Felipe con la ayuda de Gemini
# Fecha: 2025/08/11
# ============================================================================

import os
import sys
import time

def run_cmd(cmd, check_error=True):
    """Ejecuta un comando en el shell."""
    print(f"--- Ejecutando: {cmd} ---")
    status = os.system(cmd)
    if check_error and status != 0:
        print(f"\n[ERROR FATAL] El comando falló. El script se detendrá.")
        sys.exit(1)

def main():
    if os.geteuid() != 0:
        print("[ERROR] Este script debe ejecutarse con 'sudo'.")
        sys.exit(1)

    print("=== INICIANDO CONFIGURACIÓN DEL PICAR-B PARA BOOKWORM (64-BIT) ===")
    
    # --- 1. ACTUALIZACIÓN Y DEPENDENCIAS DEL SISTEMA (APT) ---
    print("\n=== PASO 1: Actualizando e instalando paquetes del sistema (APT) ===")
    run_cmd("sudo apt-get update")
    run_cmd("sudo apt-get upgrade -y")
    
    apt_packages = [
        "python3-pip", "python3-dev", "i2c-tools", "build-essential",
        "libcamera-apps", "python3-libgpiod", "python3-picamera2"
    ]
    run_cmd("sudo apt-get install -y " + " ".join(apt_packages))

    # --- 2. INSTALACIÓN DE LIBRERÍAS DE PYTHON (PIP) ---
    print("\n=== PASO 2: Instalando librerías de Python (PIP) ===")
    pip_packages = [
        "Adafruit-Blinka",
        "adafruit-circuitpython-pca9685",
        "adafruit-circuitpython-motor",
        "mpu6050-raspberrypi",
        "numpy==2.2.4", # Fijar versión por compatibilidad con OpenCV
        "opencv-contrib-python",
        "flask",
        "flask-cors",
        "websockets",
        "imutils",
        "pybase64",
        "psutil"
    ]
    run_cmd("sudo python3 -m pip install --break-system-packages --upgrade " + " ".join(pip_packages))

    # --- 3. CONFIGURACIÓN DE HARDWARE ---
    print("\n=== PASO 3: Configurando interfaces de hardware (I2C) ===")
    # Este paso suele requerir 'sudo raspi-config', pero podemos intentarlo con sed
    # Se recomienda ejecutar 'sudo raspi-config' -> 3 -> I5 para asegurar que I2C esté activo.
    run_cmd("sudo raspi-config nonint do_i2c 0")
    print("Interfaz I2C habilitada.")

    # --- 4. CONFIGURACIÓN DEL SERVICIO DE AUTOARRANQUE (SYSTEMD) ---
    print("\n=== PASO 4: Creando servicio de autoinicio (systemd) ===")
    project_path = os.getcwd() # Asume que el script se ejecuta desde la carpeta raíz del proyecto
    
    service_content = f"""[Unit]
Description=PiCar-B Auto Start Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 {project_path}/server/webServer.py
WorkingDirectory={project_path}/server
StandardOutput=journal
StandardError=journal
Restart=always
User=root

[Install]
WantedBy=multi-user.target
"""
    service_path = "/etc/systemd/system/picarb.service"
    try:
        with open(service_path, "w") as f:
            f.write(service_content)
        print(f"[OK] Fichero de servicio creado en '{service_path}'")
    except Exception as e:
        print(f"[ERROR] No se pudo crear el fichero de servicio: {e}")
        sys.exit(1)
        
    run_cmd("sudo systemctl daemon-reload")
    run_cmd("sudo systemctl enable picarb.service")
    
    print("\n" + "="*50)
    print("      ✅ ¡INSTALACIÓN COMPLETADA CON ÉXITO! ✅")
    print("="*50)
    print("\nEl servicio 'picarb.service' se iniciará en el próximo arranque.")
    print("Es necesario reiniciar para que todos los cambios surtan efecto.")
    
    reboot_choice = input("\n¿Deseas reiniciar ahora? (s/N): ")
    if reboot_choice.lower() == 's':
        print("Reiniciando en 5 segundos...")
        time.sleep(5)
        os.system("sudo reboot")
    else:
        print("\n[AVISO] Recuerda reiniciar manualmente con 'sudo reboot'.")

if __name__ == "__main__":
    main()
