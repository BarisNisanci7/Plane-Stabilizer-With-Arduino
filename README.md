
## AUTOMATIC PLANE STABILIZER WITH ARDUINO
This project presents an Arduino-based flight control system using MPU6050 accelerometer, gyroscope sensor, servo motors and Arduino microcontroller. It interprets sensor data to detect aircraft motion and executes flight control strategies accordingly. Key functions include collection of sensor data, switching to autopilot mode via a button, and flight control in automatic or manual modes. In automatic mode, servo motors are adjusted based on sensor readings, while manual mode offers direct control via analog joysticks. 

My Wokwi Simulation Link: https://wokwi.com/projects/394526468863071233

## Hardware Components

* Analog Joystick x2
* Arduino Mega
* Buzzer
* LCD_I2C (20 x 4)
* Led
* MPU6050
* Push Buton
* Servo x5


## Functions
* readMPU(): Reads and processes data from the MPU6050 sensor.
* beepBuzzer(): Controls the buzzer to produce beeps at specified frequencies and durations.
* readAnalog(): Reads values from analog pins using AVR registers.
* I2C functions: Functions for initializing and controlling I2C communication

## Constants & Variables
The system uses several integer variables to store data from the MPU6050 sensor. These variables include accelerometer and gyroscope data. Furthermore, a servo object is defined for each control surface: left and right ailerons, left and right elevators and rudder. Control flags monitor the status of the autopilot and the pushbutton. The “automaticPilot” flag determines the automatic or manual control mode, while the “previousButtonState” and “currentButtonState” variables track button interactions and switch between control modes. These constants and variables enable communication between system components and control the state of the system.


## Development Environment
If you have physical components, you can use Microchip Studio to upload project files. If you do not have physical components, another option is to use Wokwi, a simulation application.
