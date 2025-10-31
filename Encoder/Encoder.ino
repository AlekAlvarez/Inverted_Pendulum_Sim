//600 pulses per revolution
#include <ESP32Encoder.h> /*library uses pulse counter hardware (PCNT) */
#include <vector> //for time calculation
#include <sys/time.h>
//uses interrupts
//S3 only has 2 pcnt modules and can support 2 encoders, but should be enough

int currentCLKCount;
int angle;
int step; //Channel A
int direction; // Channel B

ESP32Encoder encoder; 

const int clkPIN = 10; //channel A pin
const int dtPIN = 11;  //Channel B pin

const int EnPin = 14;    //when high, motor can be manually adjusted. Otherwise, automatic
const int pulsePin = 13; //sort of a clock, where each edge equals a step
const int dirPin = 12;   //when high, moves clockwise.

const int ENCODER_PPR = 2400; //quad = 4 * 600 ppr

const int PULSE_DELAY = 500; //in microseconds

//for angular velocity approximation
std::vector<int> angles;
std::vector<int> times;
int angular_velocity = 0;

//for cart position and velocity
int xStep = 0;
int prev_x;
int velocity = 0;
//may be unecessary
int64_t get_time() {
    struct timeval tod;
    gettimeofday(&tod, NULL);
    int64_t t = (int64_t)tod.tv_sec * 1000LL + (int64_t)tod.tv_usec / 1000LL;//convert timeval to ms
    return t;
}

void removeN(std::vector<int>& angles, std::vector<int>& times, int n){
  angles.erase(angles.begin(), angles.begin() + n);
  times.erase(times.begin(), times.begin() + n);
}

void setup() {

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
  //https://www.mouser.com/ds/2/408/TB6600HG-483084.pdf
  //500millisecond setup time, and another 500 millisecond hold time

  //for angles, code can be as follows
  currentCLKCount = encoder.getCount();
  angle = currentCLKCount * (360.0 / ENCODER_PPR);

  Serial.print("Angle: ");
  Serial.println(angle);
  int time = get_time();
  angles.push_back(angle);
  times.push_back(time);

  if (angles.size() == 2) {
    angular_velocity = (angles[1]-angles[0])/(times[1]-times[0]);
    angles.erase(angles.begin());
    times.erase(times.begin()); //to keep memory small
  }
  else {
    angular_velocity = 0;
  }
  //for x position and x', refer to time
  if (times.size() == 2) {
    velocity = (xStep - prev_x)/(times[1] - times[0]);
  }
  //angular velocity calculated by luke: vector in C++; check github
  /*For the control/math itself*/
  //most likely need variable x, x', θ, θ'
  //can be found with lagrange equations or other method

  //psuedocode:
  //double error = θ - 90;
  int error = (angle - 90);

  //if error is less than 0, then angle is to the right of 90 degrees
  //right now, error is in angles, need to consider angular velocity, cart velocity, position, and angle (controls)
  prev_x = xStep;
  xStep += error;
  if (error < 0) {
    digitalWrite(dirPin, LOW);
  } else {
    digitalWrite(dirPin, HIGH);
  }

  if(error != 0 && (times[1] - times[0] > PULSE_DELAY)) {
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pulsePin, LOW);
    //if it goes right
    if (error > 0) {
      error--;
      xStep++;
    }
    else {
      error++;
      xStep--;
    }
  }
  //rough example
  for (int i = 0; i < error; i++) {
    digitalWrite(pulsePin, HIGH);
    delayMicroSeconds(500); //not feasible since whole code stops until 500 microseconds have passed
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(500);
  }
}