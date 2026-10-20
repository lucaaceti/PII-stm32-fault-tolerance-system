#!/usr/bin/env bash
set -e

FIRMWARE=${1:-build/PII-stm32-fault-tolerance-system.elf}

if [ ! -f "$FIRMWARE" ]; then
    echo "Errore: File firmware '$FIRMWARE' non trovato."
    exit 1
fi

echo "Programmazione di $FIRMWARE su STM32L152RE..."
openocd -f interface/stlink.cfg -f target/stm32l1.cfg \
    -c "program $FIRMWARE verify reset exit"
