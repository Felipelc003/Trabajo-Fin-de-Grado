#!/usr/bin/python3
# ============================================================================
# Script de Instalación Completa para Adeept PiCar-B (Versión Final Corregida)
# Adaptado para Raspberry Pi OS Bookworm y librerías modernas
# Fecha: 18/08/2025
# ============================================================================

import os
import sys
import time

def run_cmd(cmd, check_error=True):
    """Ejecuta un comando en el shell."""
    print(f"--- Ejecutando: {cmd} ---")
    status = os.system(cmd)
    if check_error and status != 0:
        print(f"\n[ERROR FATAL] El comando falló con código de salida {status}.")
        print("El script se detendrá. Revisa el error de arriba.")
        sys.exit(1)
    return status

def main():
    """Función principal del script de instalación."""
    if os.geteuid() != 0:
        print("[ERROR] Este script debe ser ejecutado con 'sudo'.")
        sys.exit(1)

    print("=== INICIANDO LA CONFIGURACIÓN DEL PICAR-B (VERSIÓN FINAL) ===")
    time.sleep(2)

    # --- 1. ACTUALIZACIÓN DEL SISTEMA ---
    print("\n=== PASO 1: Actualizando el sistema operativo ===")
    run_cmd("sudo apt-get update")
    run_cmd("sudo apt-get upgrade -y")

    # --- 2. INSTALACIÓN DE DEPENDENCIAS DEL SISTEMA (APT) ---
    print("\n=== PASO 2: Instalando dependencias del sistema (apt) ===")
    sys_packages = [
        "python3-dev", "python3-pip", "build-essential", "swig", 
        "i2c-tools", "python3-smbus", "libatlas-base-dev",
        "libcamera-dev", "libcamera-apps", "git",
        # Dependencias para create_ap
        "util-linux", "procps", "hostapd", "iproute2", "iw", "haveged", "dnsmasq",
        # Dependencias para librerías modernas de GPIO
        "python3-gpiozero", "python3-pigpio"
    ]
    run_cmd("sudo apt-get install -y " + " ".join(sys_packages))

    # --- 3. INSTALACIÓN Y ACTIVACIÓN DE PIGPIO DAEMON ---
    print("\n=== PASO 3: Configurando el servicio PIGPIO ===")
    run_cmd("sudo systemctl enable pigpiod")
    run_cmd("sudo systemctl start pigpiod")
    print("[OK] Servicio PIGPIO activado y en ejecución.")

    # --- 4. INSTALACIÓN DE LIBRERÍAS DE PYTHON (PIP) ---
    print("\n=== PASO 4: Instalando librerías de Python (pip) ===")
    py_packages = [
        "pip", "setuptools", "wheel", "adafruit-pca9685",
        "mpu6050-raspberrypi", "flask", "flask-cors",
        "websockets", "imutils", "pybase64", "psutil",
        "numpy", "opencv-contrib-python", "pyzmq", "picamera2",
        # Librería moderna para LEDs WS2812
        "adafruit-circuitpython-neopixel",
        # Librería C de bajo nivel, necesaria para neopixel
        "rpi-ws281x"
    ]
    run_cmd("sudo pip3 install --break-system-packages --upgrade " + " ".join(py_packages))
    
    # --- 5. CONFIGURACIÓN DE HARDWARE (/boot/config.txt) ---
    print("\n=== PASO 5: Configurando interfaces de hardware ===")
    boot_config_path = "/boot/config.txt" # En Bookworm la ruta es /boot/config.txt
    
    # Asegurarse de que el fichero existe
    if not os.path.exists(boot_config_path):
        print(f"[ERROR] No se encuentra el fichero de configuración en '{boot_config_path}'")
        sys.exit(1)
        
    configs_to_add = {
        "dtparam=i2c_arm=on": False,
        "start_x=1": False,
        "camera_auto_detect=1": False,
        "dtoverlay=vc4-kms-v3d": True, # True significa que debe estar comentada (#)
        "dtparam=audio=off": False # <-- ¡AÑADIDO IMPORTANTE!
    }
    
    with open(boot_config_path, 'r') as f:
        lines = f.readlines()
        
    with open(boot_config_path, 'w') as f:
        for line in lines:
            stripped_line = line.strip()
            # Eliminar comentarios de la línea para la comparación
            if '#' in stripped_line:
                clean_line = stripped_line.split('#')[0].strip()
            else:
                clean_line = stripped_line

            found = False
            for key, should_be_commented in configs_to_add.items():
                if key == clean_line:
                    if should_be_commented:
                        f.write(f"#{key}\n")
                    else:
                        f.write(f"{key}\n")
                    configs_to_add.pop(key)
                    found = True
                    break
            if not found:
                f.write(line)

    # Añadir las líneas que no se encontraron en el fichero
    with open(boot_config_path, 'a') as f:
        for key, should_be_commented in configs_to_add.items():
            if should_be_commented:
                f.write(f"#{key}\n")
            else:
                f.write(f"{key}\n")
    
    print(f"[OK] Fichero '{boot_config_path}' actualizado.")

    # --- 6. CONFIGURACIÓN DEL PUNTO DE ACCESO (AP) ---
    print("\n=== PASO 6: Configurando el modo de Punto de Acceso (create_ap) ===")
    if not os.path.exists("/home/pi/create_ap"):
        run_cmd("git clone https://github.com/oblique/create_ap /home/pi/create_ap")
        run_cmd("cd /home/pi/create_ap && sudo make install")

    # --- 7. CONFIGURACIÓN DE AUTOINICIO (SYSTEMD) ---
    print("\n=== PASO 7: Creando servicio de autoinicio (systemd) ===")
    project_root = os.path.dirname(os.path.abspath(__file__))
    
    service_content = f"""[Unit]
Description=PiCar-B Auto Start Service
After=network.target pigpiod.service
Wants=pigpiod.service

[Service]
ExecStart=/usr/bin/python3 {project_root}/server/webServer.py
WorkingDirectory={project_root}
StandardOutput=inherit
StandardError=inherit
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
    
    # --- 8. PASOS FINALES ---
    print("\n" + "="*50)
    print("      ✅ ¡INSTALACIÓN COMPLETADA CON ÉXITO! ✅")
    print("="*50)
    print("\nEs **muy importante** reiniciar el sistema ahora.")
    
    reboot_choice = input("\n¿Deseas reiniciar ahora? (s/N): ")
    if reboot_choice.lower() == 's':
        print("Reiniciando en 5 segundos...")
        time.sleep(5)
        os.system("sudo reboot")
    else:
        print("\n[AVISO] Recuerda reiniciar manualmente con 'sudo reboot'.")

if __name__ == "__main__":
    main()
