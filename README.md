# USB-to-CAN Adapter

This project is a custom USB-to-CAN adapter built around an STM32G474 microcontroller and an SN65HVD230 CAN transceiver.

It was my first custom PCB project and involved schematic design, PCB layout, soldering, firmware development, debugging, and hardware testing.

## Project Goal

The goal of the board is to act as a bridge between a computer and devices on a CAN bus.

The adapter connects to a PC over USB and allows CAN messages to be transmitted and received through the custom PCB.

## Hardware

Main components:

- STM32G474RET6
- SN65HVD230 CAN transceiver
- USB-C connector
- 3.3 V voltage regulator
- SWD programming header
- CANH / CANL connector
- Status LEDs

## Firmware

The firmware is based on the GS_USB interface so the board can operate as a USB-to-CAN adapter.

The firmware handles:

- USB communication
- CAN initialization
- CAN frame transmission
- CAN frame reception
- USB-to-CAN communication

The board was tested at a CAN bitrate of:

`1 Mbps`

## Testing

To verify the adapter, I created a small CAN network using a second STM32 board.

The test setup was:

`PS5 Controller → PC → Custom USB-to-CAN PCB → CAN Bus → STM32 → PWM → DC Motor`

The PS5 controller input was read by a Python program on the PC and converted into CAN messages.

Those CAN messages were sent through the custom USB-to-CAN adapter and received by an STM32F4 Discovery board.

The STM32 then converted the received CAN data into a PWM signal used to control the speed of a DC motor.

## What I Learned

This project gave me experience with:

- PCB schematic design
- PCB layout
- Soldering and board assembly
- STM32 firmware development
- USB device communication
- CAN bus communication
- GS_USB
- Hardware debugging
- Firmware debugging
- PWM motor control

A large part of the project involved debugging communication issues and making sure the firmware configuration matched the actual PCB routing.

## CAN Termination

The adapter does not include a fixed 120-ohm CAN termination resistor.

CAN termination should be provided at the physical ends of the CAN network.

## Demo

YouTube demonstration:

`ADD_YOUR_YOUTUBE_LINK_HERE`