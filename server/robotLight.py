#!/usr/bin/env python3
# File name   : robotLight.py
# Description : Controlador unificado con librerías modernas (gpiozero y neopixel)
import time, board, neopixel, threading, os
from gpiozero import RGBLED

class RobotLight(threading.Thread):
    def __init__(self, *args, **kwargs):
        self.led_izquierdo = self.led_derecho = None
        self.pixels = None
        try:
            self.led_izquierdo = RGBLED(red=22, green=23, blue=24)
            self.led_derecho = RGBLED(red=10, green=9, blue=25)
            print("Módulo de LEDs delanteros (gpiozero/pigpio) inicializado.")
        except Exception as e:
            print(f"ADVERTENCIA: No se pudo inicializar LEDs delanteros: {e}")
        try:
            if os.geteuid() == 0:
                self.pixels = neopixel.NeoPixel(board.D13, 16, brightness=0.8, auto_write=False)
                print("Controlador de LEDs traseros (neopixel) inicializado.")
            else:
                print("ADVERTENCIA: LEDs traseros deshabilitados (ejecución sin sudo).")
                self.pixels = None
        except Exception as e:
            print(f"ADVERTENCIA: No se pudo inicializar LEDs traseros: {e}")
            self.pixels = None

        self.lightMode = 'breath'
        self.colorBreathR, self.colorBreathG, self.colorBreathB = 0.3, 0.3, 1.0

        super(RobotLight, self).__init__(*args, **kwargs)
        self.daemon = True
        self.__flag = threading.Event()
        self.__flag.set()

    def front_all_off(self):
        if self.led_izquierdo: self.led_izquierdo.off()
        if self.led_derecho: self.led_derecho.off()

    def front_color(self, color_name):
        color_map = {'red':(0,1,1), 'blue':(1,1,0), 'green':(1,0,1), 'white':(0,0,0), 'black':(1,1,1), 'yellow':(0,0,1), 'cian':(1,0,0), 'magenta':(0,1,0)}
        if color_name in color_map:
            if self.led_izquierdo: self.led_izquierdo.color = color_map[color_name]
            if self.led_derecho: self.led_derecho.color = color_map[color_name]

    def front_turn_left(self): self.front_color('white')
    def front_turn_right(self): self.front_color('white')
    def rear_set_color(self, r, g, b):
        if self.pixels: self.pixels.fill((r, g, b)); self.pixels.show()

    def pause(self): self.lightMode = 'none'; self.front_all_off(); self.rear_set_color(0, 0, 0); self.__flag.clear()
    def resume(self): self.__flag.set()
    def police(self):
        if self.lightMode == 'police': self.breath(0.3, 0.3, 1.0)
        else: self.lightMode = 'police'; self.resume()
    def rainbow(self):
        if self.lightMode == 'rainbow': self.breath(0.3, 0.3, 1.0)
        else: self.lightMode = 'rainbow'; self.resume()
    def breath(self, r, g, b): self.lightMode = 'breath'; self.colorBreathR, self.colorBreathG, self.colorBreathB = r, g, b; self.resume()
    
    def policeProcessing(self):
        self.front_color('white')
        while self.lightMode == 'police':
            self.rear_set_color(0, 0, 255); time.sleep(0.1)
            if self.lightMode != 'police': break
            self.rear_set_color(0, 0, 0); time.sleep(0.1)
            self.rear_set_color(255, 0, 0); time.sleep(0.1)
            if self.lightMode != 'police': break
            self.rear_set_color(0, 0, 0); time.sleep(0.1)
        self.front_all_off()
    
    def breathProcessing(self):
        self.front_all_off(); breath_steps = 15.0
        while self.lightMode == 'breath':
            for i in range(int(breath_steps) + 1):
                if self.lightMode != 'breath': break
                factor = i / breath_steps; self.rear_set_color(int(self.colorBreathR*255*factor), int(self.colorBreathG*255*factor), int(self.colorBreathB*255*factor)); time.sleep(0.05)
            for i in range(int(breath_steps), -1, -1):
                if self.lightMode != 'breath': break
                factor = i / breath_steps; self.rear_set_color(int(self.colorBreathR*255*factor), int(self.colorBreathG*255*factor), int(self.colorBreathB*255*factor)); time.sleep(0.05)

    def wheel(self, pos):
        if pos < 85: return (pos * 3, 255 - pos * 3, 0)
        elif pos < 170: pos -= 85; return (255 - pos * 3, 0, pos * 3)
        else: pos -= 170; return (0, pos * 3, 255 - pos * 3)

    def rainbowProcessing(self):
        self.front_all_off()
        if not self.pixels: return
        while self.lightMode == 'rainbow':
            for j in range(255):
                if self.lightMode != 'rainbow': break
                for i in range(self.pixels.n):
                    pixel_index = (i * 256 // self.pixels.n) + j; self.pixels[i] = self.wheel(pixel_index & 255)
                self.pixels.show(); time.sleep(0.001)

    def lightChange(self):
        if self.lightMode == 'none': self.pause()
        elif self.lightMode == 'police': self.policeProcessing()
        elif self.lightMode == 'breath': self.breathProcessing()
        elif self.lightMode == 'rainbow': self.rainbowProcessing()

    def run(self):
        while True: self.__flag.wait(); self.lightChange()

    def cleanup(self):
        self.pause()
        if self.led_izquierdo: self.led_izquierdo.close()
        if self.led_derecho: self.led_derecho.close()
