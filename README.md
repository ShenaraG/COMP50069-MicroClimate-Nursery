# COMP50069-MicroClimate-Nursery
ESP32 Automated Commercial Micro-Climate Nursery
# Automated Commercial Micro-Climate Nursery

COMP50069 – Hardware, Microcontrollers and Sensors

## Scenario
Scenario 2 – Automated Commercial Micro-Climate Nursery

## Overview
This project implements an ESP32-based automated nursery system that
monitors temperature, humidity and natural light conditions and responds
using grow LEDs and a servo-controlled ventilation flap.

## Main Hardware
- ESP32
- DHT22 Temperature and Humidity Sensor
- LDR with 10 kΩ voltage divider
- 3 LEDs
- Servo Motor
- I2C OLED Display
- Manual Override Push Button

## Operating Modes
1. Autonomous Mode
2. Manual Override Mode
3. Safety Mode

Priority:
Safety > Manual Override > Autonomous

Critical Heat is handled as a high-priority condition within Autonomous Mode.

## Main Features
- Temperature and humidity monitoring
- ADC light monitoring
- Automatic grow-light control
- PWM proportional vent control
- Manual Override
- Sensor-failure Safety Mode
- UART configuration and live status
- I2C OLED monitoring
- Non-blocking timing using millis()
- GPIO interrupt for the Manual Override button

