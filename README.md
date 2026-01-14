# Pronosupinator Control Firmware (PIC18F4550)

This repository contains the firmware developed for a **pronosupinator-based assistive device**, implemented as part of an academic mechatronics project.  
The system uses a **PIC18F4550 microcontroller** to acquire sensor data, process user input, provide visual feedback, and transmit control commands wirelessly to a mobile robotic platform.

## Project Overview

The pronosupinator is designed to assist and monitor **forearm pronation–supination movements**.  
It was developed in collaboration with a rehabilitation center to support motor therapy for a pediatric user, transforming rehabilitation exercises into an interactive control interface for a mobile platform.

The firmware running on the PIC handles:

- Analog signal acquisition from sensors
- Real-time angle computation
- User interface via OLED display
- Bluetooth communication with an Arduino-based mobile platform
- Safety handling through an emergency stop mechanism

## Hardware Architecture

**Microcontroller**
- PIC18F4550 (8-bit)

**Sensors & Inputs**
- GY-61 analog accelerometer (Y-axis used for pronation/supination angle)
- KY-023 analog joystick (forward/backward control)
- Dual push buttons (emergency stop)

**Outputs & Interfaces**
- 128×64 OLED display (I²C)
- LED ring driven through a 74HC138 demultiplexer
- Bluetooth module (UART)

**Other**
- Analog signal conditioning stage using op-amps
- Custom-designed PCBs (main board, LED demux board, button board)

## Firmware Responsibilities

The firmware is responsible for:

- Configuring MCU peripherals (ADC, UART, I²C, interrupts)
- Reading and filtering analog sensor data
- Converting accelerometer readings into an angle range of ±90°
- Displaying system status, angle, and motion indicators on the OLED
- Sending joystick and angle data via Bluetooth (UART)
- Handling emergency stop logic using interrupts
- Managing LED indicators based on angular position

## Main Features

- **ADC-based sensor acquisition** (accelerometer and joystick)
- **OLED graphical interface** with real-time updates
- **Bluetooth UART communication** with a mobile platform
- **Emergency stop system** implemented via interrupts
- **Angle-based LED feedback** using a demultiplexer
- Modular and readable firmware structure

## Development Environment

- **IDE:** MPLAB X
- **Compiler:** XC8
- **Language:** C
- **Clock frequency:** 16 MHz
- **Target MCU:** PIC18F4550

## Repository Contents

This repository contains **only the firmware source file** used in the project:

- `Prono.c` – Main firmware file for the pronosupinator system

No schematics, PCB files, or mechanical designs are included in this repository.

## Notes

- The firmware is hardware-specific and depends on the exact pinout and external circuitry described in the project.
- This code was developed for academic and experimental purposes.
- Adaptations may be required to reuse it on different hardware configurations.

## Authors

Developed by the *Pan de Muerto* team as part of an academic project in Mechatronics Engineering.

## License

This project is shared for educational purposes.  
No warranty is provided; use at your own risk.
