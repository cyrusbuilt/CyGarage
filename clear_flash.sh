#!/bin/bash

FLASHER="$( which esptool.py )"
if [[ -z "${FLASHER}" ]]; then
    echo "WARN: Flash tool not installed."
    read -rp $'Do you want to install now? (Y/n): ' choice
    if [[ "${choice}" = "y" || "${choice}" = "Y" ]]; then
        pip install esptool
        FLASHER="$( which esptool.py )"
        if [[ -z "${FLASHER}" ]]; then
            echo "ERROR: Installation failed."
            exit 1
        fi
    else
        exit 0
    fi
fi

${FLASHER} -p /dev/cu.usbserial-AL05HSL2 erase_flash