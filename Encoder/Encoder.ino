//600 pulses per revolution
#include <ESP32Encoder.h> /*library uses pulse counter hardware (PCNT) */
//uses interrupts
//S3 only has 2 pcnt modules and can support 2 encoders, but should be enough

int currentCLKCount;
int angle;
int step; //Channel A
int direction; // Channel B

const int clkPIN = 10; //channel A pin
const int dtPIN = 15;  //for motor driver

const int EnPin = 17;    //when high, motor can be manually adjusted. Otherwise, automatic
const int pulsePin = 19; //sort of a clock, where each edge equals a step
const int dirPin = 21;   //when high, moves clockwise.

const int ENCODER_PPR = 2400; //quad = 4 * 600 ppr

void setup() {
  ESP32Encoder encoder; //declaration
  encoder.clearCount();

  //configure pins
  //clk pin: when it rotates
  pinMode(clkPIN, INPUT_PULLUP);
  //DT pin: direction, compares itself to clk
  pinMode(dtPIN, INPUT_PULLUP); //pullup resistor

  //tb6600 receives output from esp32
  pinMode(EnPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  //for debugging; can be used to directly print values
  Serial.begin(115200);
  encoder.attachFullQuad(clkPIN, dtPIN); //uses falling and rising edges of pins to determine rotation
}

void loop() {
  //verify that encoder is 600P/R
  //stepper motor: manual movement or library?
  //https://www.mouser.com/ds/2/408/TB6600HG-483084.pdf
  //500millisecond setup time, and another 500 millisecond hold time
  //ex. below is one step in a counterclockwise direction, left

  //for angles, code can be as follows
  currentCLKCount = encoder.getCount();
  angle = currentCLKCount * (360.0 / ENCODER_PPR);

  Serial.print("Angle: ");
  Serial.println(angle);

  /*For the control/math itself*/
  //most likely need variable x, x', θ, θ'
  //can be found with lagrange equations or other method

  //psuedocode:
  //double error = θ - 90;
  int error = (angle - 90);

  //if error is less than 0, then angle is to the right of 90 degrees
  if (error < 0) {
    digitalWrite(dirPin, LOW);
    error = -error;
  } else {
    digitalWrite(dirPin, HIGH);
  }

  //rough example
  for (int i = 0; i < error; i++) {
    digitalWrite(pulsePin, HIGH);
    delay(500);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(500);
  }
}