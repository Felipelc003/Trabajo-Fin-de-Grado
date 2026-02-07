#!/usr/bin/env python3
# Nombre del archivo: base_camera.py
# Descripción: Implementación de una clase base abstracta para la gestión eficiente del streaming de video.
# Este módulo maneja la captura de frames en un hilo independiente y su distribución a múltiples clientes simultáneos.

import time
import threading
from threading import get_ident

class CameraEvent:
    """
    Gestor de eventos de sincronización para clientes de video.
    
    Esta clase permite que múltiples clientes (hilos) esperen a que un nuevo frame esté disponible.
    Utiliza un diccionario para rastrear el estado de cada cliente individualmente.
    """
    def __init__(self):
        # Diccionario que almacena eventos por identificador de hilo (thread ID).
        self.events = {}

    def wait(self):
        """
        Bloquea la ejecución del cliente actual hasta que se notifica la llegada de un nuevo frame.
        
        Si es la primera vez que el cliente solicita un frame, se crea un nuevo evento de sincronización.
        Retorna:
            True si el evento fue activado, False si hubo un timeout (dependiendo de la implementación de threading.Event).
        """
        ident = get_ident()
        if ident not in self.events:
            # Crea una nueva entrada para este cliente: [Objeto Evento, Timestamp de última actividad]
            self.events[ident] = [threading.Event(), time.time()]
        return self.events[ident][0].wait()

    def set(self):
        """
        Notifica a todos los clientes conectados que un nuevo frame está listo para ser procesado.
        
        Itera sobre todos los clientes registrados, activa sus eventos y actualiza su timestamp.
        También realiza una limpieza automática de clientes inactivos (timeout > 5 segundos) para liberar recursos.
        """
        now = time.time()
        remove = []
        for ident, event in self.events.items():
            if not event[0].is_set():
                # Si el evento no estaba activado, lo activamos y actualizamos el tiempo.
                event[0].set()
                event[1] = now
            else:
                # Si el evento ya estaba activado y ha pasado el tiempo límite, se marca para eliminación.
                # Esto sucede si el cliente no ha consumido el evento anterior (ha cerrado la conexión o es lento).
                if now - event[1] > 5:
                    remove.append(ident)

        # Eliminar clientes inactivos del registro
        for ident in remove:
            del self.events[ident]

    def clear(self):
        """
        Resetea el evento de señalización para el cliente actual.
        
        Debe llamarse después de procesar un frame para prepararse para la siguiente espera (wait).
        """
        ident = get_ident()
        if ident in self.events:
            self.events[ident][0].clear()

class BaseCamera:
    """
    Clase base abstracta para cámaras de streaming.
    
    Implementa el patrón Singleton para el hilo de captura de fondo. Esto asegura que solo haya un proceso capturando imágenes de la cámara física, independientemente de cuántos clientes web estén visualizando el stream.
    """
    thread = None       # Referencia al hilo de fondo único (background thread)
    frame = None        # Almacena el último frame capturado
    last_access = 0     # Timestamp del último acceso de cualquier cliente
    event = CameraEvent() # Instancia compartida del gestor de eventos

    def __init__(self):
        """
        Constructor de la clase.
        
        Verifica si el hilo de captura está activo. Si no lo está, inicia un nuevo hilo
        que ejecutará el método `_thread`. Bloquea la inicialización hasta que el primer
        frame válido haya sido capturado para evitar errores en el cliente.
        """
        if BaseCamera.thread is None:
            BaseCamera.last_access = time.time()
            # Inicio del hilo demonio (daemon) implícito al no hacer join, encargado de la captura.
            BaseCamera.thread = threading.Thread(target=self._thread)
            BaseCamera.thread.start()

            # Espera activa (bloqueo) hasta asegurar que hay datos de imagen disponibles.
            while self.get_frame() is None:
                time.sleep(0)

    def get_frame(self):
        """
        Método principal para obtener la imagen actual.
        
        Marca la actividad del cliente, espera la notificación de nuevo frame (wait),
        limpia la señal (clear) y retorna los datos de la imagen.
        """
        BaseCamera.last_access = time.time()
        
        # Sincronización: esperar a que el hilo de fondo capture una nueva imagen.
        BaseCamera.event.wait()
        BaseCamera.event.clear()
        
        return BaseCamera.frame

    @staticmethod
    def frames():
        """
        Generador abstracto de frames.
        
        Su responsabilidad es interactuar con el hardware (OpenCV, PiCamera, etc.) y yield (ceder) los bytes de cada imagen capturada.
        """
        raise RuntimeError('El método estático frames() debe ser implementado por la subclase.')

    @classmethod
    def _thread(cls):
        """
        Lógica del hilo de fondo (Background Thread).
        
        Obtiene el iterador del generador `frames()` y entra en un bucle continuo.
        Para cada frame generado:
        1. Actualiza la variable de clase `frame`.
        2. Notifica a todos los clientes esperando (`event.set()`).
        
        El hilo se detendrá automáticamente si no hay clientes activos durante un periodo.
        """
        print('[BaseCamera] Hilo de captura iniciado.')
        frames_iterator = cls.frames()
        for frame in frames_iterator:
            BaseCamera.frame = frame
            BaseCamera.event.set()
            time.sleep(0) # Cede el control brevemente para permitir cambios de contexto.
            
        BaseCamera.thread = None