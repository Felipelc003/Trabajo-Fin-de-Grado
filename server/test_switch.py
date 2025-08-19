import switch
import time

print("Iniciando prueba directa del controlador switch.py")
print("-------------------------------------------------")

try:
    # Inicializar el hardware
    switch.switchSetup()
    print("Switch Setup... OK")

    # Apagar todos los puertos al inicio
    switch.set_all_switch_off()
    print("Apagando todos los puertos...")
    time.sleep(2)

    # Probar cada puerto individualmente
    for i in range(1, 4):
        print(f"Probando Puerto {i}: ON")
        switch.switch(i, 1) # Encender puerto
        time.sleep(2)
        print(f"Probando Puerto {i}: OFF")
        switch.switch(i, 0) # Apagar puerto
        time.sleep(1)

    print("-------------------------------------------------")
    print("Prueba finalizada. Observa si las luces delanteras respondieron.")

except Exception as e:
    print(f"\nOcurrió un error durante la prueba: {e}")

