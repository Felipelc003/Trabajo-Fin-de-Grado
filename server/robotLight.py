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
# Nombre del archivo: robotLight.py
# Descripción: Módulo de control para el sistema de iluminación frontal (LEDs RGB).
# Utiliza la librería 'gpiozero' para una gestión simplificada de los estados de los pines GPIO.

from gpiozero import RGBLED

class RobotLight:
    """
    Clase para el control de los LEDs RGB frontales del robot.
    
    Esta clase abstrae la complejidad del manejo de pines GPIO, permitiendo encender, 
    apagar y cambiar el color de los LEDs mediante métodos de alto nivel.
    """
    def __init__(self):
        """
        Inicializa la instancia de RobotLight y configura los pines GPIO.
        
        Se definen dos objetos RGBLED (izquierdo y derecho) asignados a pines específicos de la Raspberry Pi.
        Incluye manejo de excepciones para evitar que fallos en el hardware detengan la ejecución del programa.
        """
        self.led_izquierdo = None
        self.led_derecho = None
        try:
            # Configuración de pines GPIO para los LEDs (Esquema BCM).
            # LED Izquierdo: Rojo=22, Verde=23, Azul=24
            self.led_izquierdo = RGBLED(red=22, green=23, blue=24)
            # LED Derecho: Rojo=10, Verde=9, Azul=25
            self.led_derecho = RGBLED(red=10, green=9, blue=25)
            print("[RobotLight] LEDs delanteros inicializados correctamente.")
        except Exception as e:
            print(f"[RobotLight] Advertencia: Fallo en inicialización de LEDs: {e}")

    def front_all_off(self):
        """
        Apaga ambos LEDs delanteros inmediatamente.
        """
        if self.led_izquierdo: self.led_izquierdo.off()
        if self.led_derecho: self.led_derecho.off()

    def front_color(self, color_name):
        """
        Establece un color específico para ambos LEDs delanteros.
        
        Parámetros:
        - color_name (str): Nombre del color deseado (ej. 'red', 'blue', 'white').
        
        Nota: El mapa de colores utiliza valores (R, G, B). Dependiendo de si los LEDs son 
        de ánodo o cátodo común, estos valores pueden requerir inversión (0=Encendido vs 1=Encendido).
        El mapa actual asume una lógica invertida (Active LOW) donde 0 enciende el componente.
        """
        color_map = {
            'red': (0, 1, 1), 'blue': (1, 1, 0), 'green': (1, 0, 1),
            'white': (0, 0, 0), 'black': (1, 1, 1), 'yellow': (0, 0, 1),
            'cian': (1, 0, 0), 'magenta': (0, 1, 0)
        }
        
        if color_name in color_map:
            val = color_map[color_name]
            if self.led_izquierdo: self.led_izquierdo.color = val
            if self.led_derecho: self.led_derecho.color = val

    # --- Métodos de Señalización (Alias) ---
    def front_turn_left(self): 
        """Indica giro a la izquierda (actualmente color blanco estático)."""
        self.front_color('cian')
        
    def front_turn_right(self): 
        """Indica giro a la derecha (actualmente color blanco estático)."""
        self.front_color('magenta')

    # --- Métodos de Compatibilidad ---
    # Estos métodos se mantienen vacíos para preservar la compatibilidad con llamadas 
    # de versiones anteriores del software o interfaces que esperan estas funciones.
    def start(self): pass
    def breath(self, *args): pass
    def police(self): pass
    def rainbow(self): pass
    def rear_set_color(self, *args): pass

    def cleanup(self):
        """
        Libera los recursos de hardware y cierra las conexiones GPIO de manera segura.
        Debe ser llamado antes de terminar el programa.
        """
        self.front_all_off()
        if self.led_izquierdo: self.led_izquierdo.close()
        if self.led_derecho: self.led_derecho.close()