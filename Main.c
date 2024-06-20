#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "I2C.h"
#define MPU 0x68

// Variables for MPU6050 sensor
int16_t x_axis, y_axis, z_axis, x_gyro, y_gyro, z_gyro, temperature;

// Define lcd screen
LiquidCrystal_I2C lcd(0x27,16,2);

// Servo objects for Ailerons, Elevators and Rudder
Servo left_aileron, right_aileron, left_elevator, right_elevator, rudder;

bool automaticPilot = false;
bool previousButtonState = false;

void setup() {
  // Initialize I2C communication and start MPU6050 sensor
  i2c_init();
  i2c_start();
  i2c_write(MPU << 1);
  i2c_write(0x6B);
  i2c_write(0);
  i2c_stop();

  // Initialize LCD screen
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.print("System Starting...");
  delay(2000);

  // set led pin as output
  DDRA |= (1 << PA0);
  // set buzzer pin as output
  DDRA |= (1 << PA3);
  // set button pin as input
  DDRA &= ~(1 << PA1);
  //PORTA |= (1 << PA1);

  // Initialize ADC
  ADMUX = (1 << REFS0); // Reference voltage set to AVcc
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC and set prescaler to 128

  // Define pins of servos
  left_aileron.attach(35);
  right_aileron.attach(37);
  left_elevator.attach(39);
  right_elevator.attach(41);
  rudder.attach(43);
}

void loop() {
  // Read the state of the button
  bool currentButtonState  = PINA & (1 << PA1);
  // Read data from MPU6050 sensor
  readMPU();

  // Check if the MPU6050 sensor values are not zero
  if(!(abs(x_axis) == 0 && abs(y_axis) == 0 && abs(z_axis) == 0)) {
    PORTA |= (1 << PA0); // Turn on LED
    beepBuzzer(70, 100); // Beep the buzzer if values are not zero
  } else {
    PORTA &= ~(1 << PA0);
  }

  // Toggle automaticPilot state when button is pressed
  if (currentButtonState && !previousButtonState) {
      automaticPilot = !automaticPilot;
  }
  previousButtonState = currentButtonState;

  // Control logic based on automaticPilot state
  switch (automaticPilot) {
    case 1:
      // Automatic pilot is on
      lcd.setCursor(0, 0);
      lcd.print("Automatic Control ");

      // Control servos based on MPU6050 sensor values
      left_aileron.write(map(x_axis, -12000, 12000, 0, 180));
      right_aileron.write(map(x_axis, -12000, 12000, 180, 0));
      left_elevator.write(map(y_axis, -12000, 12000, 0, 180));
      right_elevator.write(map(y_axis, -12000, 12000, 0, 180));
      rudder.write(map(z_axis, -12000, 12000, 0, 180));
      break;

    case 0:
      // Automatic pilot is off, manual control
      lcd.setCursor(0, 0);
      lcd.print("Manual Control    ");

      // Read joystick values using direct ADC access
      uint16_t joy1_vert = readAnalog(0); // A0
      uint16_t joy1_horz = readAnalog(1); // A1
      uint16_t joy2_vert = readAnalog(3); // A3
      uint16_t joy2_horz = readAnalog(4); // A4

      // Map joystick values to servo positions
      left_aileron.write(map(joy1_horz, 0, 1023, 0, 180));
      right_aileron.write(map(joy1_horz, 0, 1023, 180, 0));
      left_elevator.write(map(joy1_vert, 0, 1023, 0, 180));
      right_elevator.write(map(joy1_vert, 0, 1023, 0, 180));
      rudder.write(map(joy2_horz, 0, 1023, 0, 180));
      break;
  }

  // Display MPU6050 sensor values on LCD
  lcd.setCursor(0, 1);
  lcd.print("X axis(ROLL):");
  lcd.print(constrain(map(x_axis, -32768, 32767, -90, 90), -180, 180));

  lcd.setCursor(0, 2);
  lcd.print("Y axis(PITCH):");
  lcd.print(constrain(map(y_axis, -32768, 32767, -90, 90), -180, 180));

  lcd.setCursor(0, 3);
  lcd.print("Z axis(YAW):");
  lcd.print(constrain(map(z_axis, -32768, 32767, -90, 90), -180, 180));

}


void readMPU() { // Function to read data from MPU6050 sensor
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);

  x_gyro = Wire.read()<<8|Wire.read();
  y_gyro = Wire.read()<<8|Wire.read();
  z_gyro = Wire.read()<<8|Wire.read();
  temperature = Wire.read()<<8|Wire.read(); // I should read this data to be sure that values comes correct order
  x_axis = Wire.read()<<8|Wire.read();
  y_axis = Wire.read()<<8|Wire.read();
  z_axis = Wire.read()<<8|Wire.read();
}


// Function to beep the buzzer at a specified frequency and duration
void beepBuzzer(unsigned int frequency, unsigned long duration) {
  unsigned long period = 1000000L / frequency;
  unsigned long cycles = frequency * duration / 1000;

  for (unsigned long i = 0; i < cycles; i++) {
    PORTA |= (1 << PA3);
    _delay_us(period / 2);
    PORTA &= ~(1 << PA3);
    _delay_us(period / 2);
  }
}

// Function to read from an analog pin using AVR registers
uint16_t readAnalog(uint8_t pin) {
  ADMUX = (ADMUX & 0xF0) | (pin & 0x0F); // Select the corresponding channel 0~7
  ADCSRA |= (1 << ADSC); // Start the conversion
  while (ADCSRA & (1 << ADSC)); // Wait for the conversion to finish
  return ADCW; // Read the ADC value
}

