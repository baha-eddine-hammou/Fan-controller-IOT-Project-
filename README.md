Overview
This project integrates a smart fan controller and a cloud-connected robotic system, demonstrating a comprehensive IoT solution that merges local control with cloud-based data management. The project is divided into two main components:

Fan Controller: Utilizes an STM32F4 microcontroller to control three fans using potentiometers and transmit data to a smartphone via Bluetooth.
Data-Sending Robot: Employs an STM32F4 microcontroller to control a robot with four motors. The robot receives data from the fan controller hub via Bluetooth and sends it to the ThingSpeak cloud platform using an ESP8266 Wi-Fi module.
Repository Structure
main_client: This file contains the code for the robot part, including motor control and data transmission to the cloud.
main_serveur: This file contains the code for the fan controller part, including fan speed control via potentiometers and Bluetooth communication with the robot.
