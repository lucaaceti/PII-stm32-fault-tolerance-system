#!/usr/bin/env python3
import sys
import time
import subprocess
import serial

PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0674FF525056805087063010-if02"
BAUD = 115200
TIMEOUT = 10  # Secondi massimi prima del timeout

# 1. Flash del firmware tramite lo script OpenOCD
print("[CI/HIL] Avvio flash del target...")
flash_res = subprocess.run(["./flash.sh"])
if flash_res.returncode != 0:
    print("[CI/HIL] Errore critico: flash fallito.")
    sys.exit(1)

# 2. Connessione alla porta seriale virtuale
print("[CI/HIL] Connessione a UART in ascolto dei log...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"[CI/HIL] Errore apertura seriale: {e}")
    sys.exit(1)

# Pulisce eventuali byte residui nel buffer
ser.reset_input_buffer()

start_time = time.time()
while time.time() - start_time < TIMEOUT:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(f"[MCU]: {line}")
        # Condizioni di successo o fallimento (personalizzabili in base al firmware)
        if "TEST_SUCCESS" in line or "BOOT_OK" in line:
            print("[CI/HIL] Esito POSITIVO: verifica completata sul silicio.")
            ser.close()
            sys.exit(0)
        if "TEST_FAIL" in line or "HARD_FAULT" in line:
            print("[CI/HIL] Esito NEGATIVO: fallimento rilevato.")
            ser.close()
            sys.exit(1)

print("[CI/HIL] Timeout: nessun messaggio di conferma entro il tempo limite.")
ser.close()
sys.exit(1)
