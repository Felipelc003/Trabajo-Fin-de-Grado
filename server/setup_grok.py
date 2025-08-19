#!/usr/bin/python3
# setup.py actualizado para Raspberry Pi OS Bookworm 64-bit (PiCar-B), sin venv
# Autor: Adaptado por Grok con base en correcciones para compatibilidad
# Fecha: 2025-08-09

import os
import time
import subprocess
import platform

def run_cmd(cmd, retries=3):
    for _ in range(retries):
        if os.system(cmd) == 0:
            return True
    print(f"[ERROR] No se pudo ejecutar: {cmd}")
    return False

def replace_num(file, initial, new_num):
    newline = ""
    str_num = str(new_num)
    with open(file, "r") as f:
        for line in f.readlines():
            if line.find(initial) == 0:
                line = str_num + '\n'
            newline += line
    with open(file, "w") as f:
        f.writelines(newline)

# Verificar arquitectura 64-bit
if platform.machine() != 'aarch64':
    print("[ERROR] Este script está optimizado para Raspberry Pi OS 64-bit (aarch64). Ejecutando en otra arquitectura podría fallar.")
    exit(1)

print("=== ACTUALIZANDO SISTEMA ===")
run_cmd("sudo apt-get update")
run_cmd("sudo apt-get -y upgrade")
run_cmd("sudo apt-get purge -y wolfram-engine libreoffice*")
run_cmd("sudo apt-get -y autoremove")
run_cmd("sudo apt-get -y clean")

print("=== INSTALANDO DEPENDENCIAS DEL SISTEMA ===")
sys_packages = [
    "python3-dev", "python3-pip", "python3-numpy", "python3-opencv",
    "libfreetype6-dev", "libjpeg-dev", "build-essential", "swig",
    "portaudio19-dev", "python3-all-dev", "python3-pyaudio", "flac",
    "i2c-tools", "python3-smbus",
    "libatlas-base-dev", "libhdf5-dev", "libhdf5-serial-dev",
    "libqt5gui5", "libqt5test5",
    "libopenblas-dev", "liblapack-dev", "libcap-dev",
    "libcamera-dev", "libcamera-apps", "libxkbcommon0",
    "libgles2-mesa", "libegl1-mesa", "libdrm2", "libgbm1"
]
run_cmd("sudo apt-get install -y " + " ".join(sys_packages))

print("=== INSTALANDO LIBRERÍAS DE PYTHON (GLOBAL) ===")
py_packages = [
    "pip", "setuptools", "wheel",
    "luma.oled", "adafruit-pca9685", "rpi_ws281x", "mpu6050-raspberrypi",
    "flask", "flask-cors", "websockets", "imutils", "pybase64", "psutil",
    "SpeechRecognition", "pyaudio", "numpy", "opencv-contrib-python",  # Sin versión fija para compatibilidad 64-bit
    "zmq", "picamera2"
]
run_cmd("sudo pip3 install --upgrade --break-system-packages " + " ".join(py_packages))

print("=== CONFIGURANDO I2C Y CÁMARA ===")
replace_num("/boot/config.txt", "#dtparam=i2c_arm=on", "dtparam=i2c_arm=on\nstart_x=1\n")
os.system("sudo raspi-config nonint do_i2c 0")
os.system("sudo raspi-config nonint do_camera 0")

print("=== INSTALANDO create_ap PARA MODO AP ===")
if not os.path.exists("/home/pi/create_ap"):
    run_cmd("git clone https://github.com/oblique/create_ap /home/pi/create_ap")
    run_cmd("cd /home/pi/create_ap && sudo make install")
run_cmd("sudo apt-get install -y util-linux procps hostapd iproute2 iw haveged dnsmasq")

print("=== CONFIGURANDO AUTOINICIO CON SYSTEMD ===")
service_file = """[Unit]
Description=PiCar-B Auto Start
After=network.target

[Service]
ExecStart=/usr/bin/python3 {path}/server/webServer.py
WorkingDirectory={path}
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
""".format(path=os.path.dirname(os.path.realpath(__file__)))

with open("/tmp/picarb.service", "w") as f:
    f.write(service_file)
run_cmd("sudo mv /tmp/picarb.service /etc/systemd/system/picarb.service")
run_cmd("sudo systemctl daemon-reload")
run_cmd("sudo systemctl enable picarb.service")

print("=== DESHABILITANDO AUDIO INTEGRADO ===")
os.system("echo 'blacklist snd_bcm2835' | sudo tee /etc/modprobe.d/snd-blacklist.conf")

print("=== INSTALACIÓN COMPLETA ===")
print("Reiniciando en 5 segundos...")
time.sleep(5)
os.system("sudo reboot")
