#!/usr/bin/python3
# ============================================================================
# Script de Instalación Completa para Adeept PiCar-B
# Adaptado para Raspberry Pi OS Bookworm (64-bit)
# fecha: 9/8/2025 20:43
# Ejecución:
# 1. Guarda este fichero como 'setup.py' en tu directorio de proyecto.
# 2. Dale permisos de ejecución: chmod +x setup.py
# 3. Ejecútalo con sudo: sudo ./setup.py
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
    # --- 0. VERIFICACIÓN INICIAL ---
    if os.geteuid() != 0:
        print("[ERROR] Este script debe ser ejecutado con privilegios de superusuario.")
        print("Por favor, ejecútalo usando 'sudo'. Ejemplo: sudo python3 setup.py")
        sys.exit(1)

    print("=== INICIANDO LA CONFIGURACIÓN DEL PICAR-B PARA BOOKWORM (64-BIT) ===")
    time.sleep(2)

    # --- 1. LIMPIEZA DE PAQUETES CONFLICTIVOS ---
    print("\n=== PASO 1: Limpiando paquetes potencialmente conflictivos ===")
    # Usamos '|| true' para que el script no falle si los paquetes no existen
    run_cmd("sudo apt-get remove --purge -y python3-opencv python3-numpy || true", check_error=False)
    run_cmd("sudo apt-get autoremove -y", check_error=False)
    run_cmd("sudo pip3 uninstall -y --break-system-packages numpy opencv-contrib-python opencv-python || true", check_error=False)
    
    # --- 2. ACTUALIZACIÓN DEL SISTEMA ---
    print("\n=== PASO 2: Actualizando el sistema operativo ===")
    run_cmd("sudo apt-get update")
    run_cmd("sudo apt-get -y upgrade")
    run_cmd("sudo apt-get purge -y wolfram-engine libreoffice*")
    run_cmd("sudo apt-get -y clean")

    # --- 3. INSTALACIÓN DE DEPENDENCIAS DEL SISTEMA (APT) ---
    print("\n=== PASO 3: Instalando dependencias base del sistema ===")
    sys_packages = [
        "python3-dev", "python3-pip", "libfreetype6-dev", "libjpeg-dev", 
        "build-essential", "swig", "portaudio19-dev", "python3-all-dev", 
        "python3-pyaudio", "flac", "i2c-tools", "python3-smbus",
        "libatlas-base-dev", "libhdf5-dev", 
        "libqt5gui5", "libqt5test5", "libopenblas-dev", "liblapack-dev", 
        "libcap-dev", "libcamera-dev", "libcamera-apps"
    ]
    run_cmd("sudo apt-get install -y " + " ".join(sys_packages))

    # --- 4. INSTALACIÓN DE LIBRERÍAS DE PYTHON (PIP) ---
    print("\n=== PASO 4: Instalando librerías de Python ===")
    py_packages = [
        "pip", "setuptools", "wheel", "luma.oled", "adafruit-pca9685", 
        "rpi_ws281x", "mpu6050-raspberrypi", "flask", "flask-cors", 
        "websockets", "imutils", "pybase64", "psutil", "SpeechRecognition", 
        "pyaudio", "numpy", "opencv-contrib-python", "pyzmq", "picamera2", "simplejpeg"
    ]
    run_cmd("sudo pip3 install --break-system-packages --upgrade " + " ".join(py_packages))
    
    # --- 5. CONFIGURACIÓN DE HARDWARE (I2C, CÁMARA) ---
    print("\n=== PASO 5: Configurando interfaces de hardware (I2C y Cámara) ===")
    boot_config_path = "/boot/firmware/config.txt"
    config_changes = {
        'dtparam=i2c_arm=on': '#dtparam=i2c_arm=on',
        'start_x=1': '#start_x=1',
        'camera_auto_detect=0': 'camera_auto_detect=1',
        'dtoverlay=vc4-kms-v3d': '#dtoverlay=vc4-kms-v3d',
        'dtoverlay=ov5647': '#dtoverlay=ov5647' # Para la cámara v1.3 del kit
    }
    
    try:
        with open(boot_config_path, 'r') as f:
            lines = f.readlines()
        
        with open(boot_config_path, 'w') as f:
            for line in lines:
                found = False
                for key, comment in config_changes.items():
                    if key in line or comment in line:
                        f.write(key + '\n')
                        config_changes.pop(key)
                        found = True
                        break
                if not found:
                    f.write(line)
            
            # Añadir las configuraciones que no se encontraron
            for key in config_changes:
                f.write(key + '\n')
        print(f"[OK] Fichero '{boot_config_path}' actualizado.")

    except Exception as e:
        print(f"[ERROR] No se pudo editar '{boot_config_path}': {e}")

    # --- 6. CONFIGURACIÓN DEL PUNTO DE ACCESO (AP) ---
    print("\n=== PASO 6: Configurando el modo de Punto de Acceso (create_ap) ===")
    if not os.path.exists("/home/pi/create_ap"):
        run_cmd("git clone https://github.com/oblique/create_ap /home/pi/create_ap")
        run_cmd("cd /home/pi/create_ap && sudo make install")
    run_cmd("sudo apt-get install -y util-linux procps hostapd iproute2 iw haveged dnsmasq")

    # --- 7. CONFIGURACIÓN DE AUTOINICIO (SYSTEMD) ---
    print("\n=== PASO 7: Creando servicio de autoinicio (systemd) ===")
    # Asume que el script a ejecutar es 'webServer.py' y está en 'server/'
    # La ruta de trabajo será la carpeta que contiene la carpeta 'server'
    project_root = os.path.dirname(os.path.abspath(__file__))
    
    service_content = f"""[Unit]
Description=PiCar-B Auto Start Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 {project_root}/server/webServer.py
WorkingDirectory={project_root}
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi
Group=pi

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
    print("\n=== PASO 8: Finalizando la instalación ===")
    run_cmd("echo 'blacklist snd_bcm2835' | sudo tee /etc/modprobe.d/snd-blacklist.conf > /dev/null")
    print("[OK] Audio integrado deshabilitado para evitar conflictos.")

    print("\n" + "="*50)
    print("      ✅ ¡INSTALACIÓN COMPLETADA CON ÉXITO! ✅")
    print("="*50)
    print("\nEl servicio 'picarb.service' se iniciará automáticamente en el próximo arranque.")
    print("Es **muy importante** reiniciar el sistema ahora para que todos los")
    print("cambios, especialmente los de hardware, surtan efecto.")
    
    reboot_choice = input("\n¿Deseas reiniciar ahora? (s/N): ")
    if reboot_choice.lower() == 's':
        print("Reiniciando en 5 segundos...")
        time.sleep(5)
        os.system("sudo reboot")
    else:
        print("\n[AVISO] Recuerda reiniciar manualmente con 'sudo reboot'.")

if __name__ == "__main__":
    main()
