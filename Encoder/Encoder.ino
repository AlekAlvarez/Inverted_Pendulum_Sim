//600 pulses per revolution
#include <ESP32Encoder.h> 
/*library uses pulse counter hardware (PCNT)  */
//uses interrupts
//S3 only has 2 pcnt modules and can support 2 encoders, but should be enough
int currentCLKCount;
int angle;

int step; //Channel A
int direction; // Channel B

const int ENCODER_PPR = 600;
void setup() {
   //configure pins
   //clk pin: when it rotates
   //DT pin: direction, compares itself to clk
   //ex: pinMode(2, INPUT_PULLUP); DT w pullup resistor
   //for debugging; can be used to directly print values
   Serial.begin(115200);
   //encoder.attachFullQuad(stepper_motor_pin, rotary_encoder_pin)
}

void loop() {
   //verify that encoder is 600P/R
   //stepper motor
   //use encoder.getCount(); function
   //for angles, code can be as follows
   currentCLKCount = encoder.getCount();
   angle = currentCLKCount * (360.0/ENCODER_PPR);
   Serial.print("Angle: ");
   Serial.println(angle);
   /*For the control/math itself*/
}