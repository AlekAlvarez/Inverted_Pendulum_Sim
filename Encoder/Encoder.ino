//600 pulses per revolution
#include <ESP32Encoder.h> /*library uses pulse counter hardware (PCNT) */
#include <vector> //for time calculation
#include <sys/time.h>
//uses interrupts
//S3 only has 2 pcnt modules and can support 2 encoders, but should be enough
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
long long currentCLKCount;
double angle;
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

//Beginnings of PID (Proportion, integral, and derivative): may omit integral for now
//if integral used, have a clamping term to prevent overshooting/cumulation of error
const double targetAngle = 90; //initial error and target
//P = Kp * e(t)
const double Kp = 1.0;
//D = k_d * d(error)/dt
const double Kd = 1.0;
int steps=0;
//for angular velocity approximation
std::vector<double> angles;
std::vector<long long> times;
double angular_velocity = 0;

//for cart position and velocity
volatile long long xStep = 0; //mechanical system: changes can randomly happen, volatile flag ensures true value is always known
long long prev_x;
int velocity = 0;
typedef struct {
    int steps;
    double error;
} DataPacket_t;
// Handle for the Queue used to send the DataPacket_t from Main Thread (Producer) to Consumer Task
QueueHandle_t calculation_queue = NULL;
// Handle for the Queue used to send the single integer result BACK from Consumer Task to Main Thread (Producer)
QueueHandle_t result_queue = NULL;
void performAcuation(void *pvParameters){
    double error = 0;
    int steps = 0;
    DataPacket_t received_packet;

    // The consumer task runs perpetually
    while (1) {
        ESP_LOGI(TAG, "Consumer: Waiting for a new Data Packet (two values) from Producer...");

        // 1. RECEIVE DataPacket_t from Producer
        if (xQueueReceive(calculation_queue, &received_packet, portMAX_DELAY) == pdPASS) {

            // Reset result calculation for the new cycle
            error=received_packet.error;
            xStep+=error;
            //while the microcontroller is collecting data and pushing an output, it needs to constantly be doing operations while knowing what the values are.
            //that means that the value of a driver cannot be manually pushed as seen below in a for loop, since the mcu does not know what is going on while it is in the loop
            //non blocking allows loop code to run continually while still generating pulses
            //Will be finished by next week!
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
              delayMicroseconds(500); //not feasible since whole code stops until 500 microseconds have passed
              digitalWrite(pulsePin, LOW);
              delayMicroseconds(500);
            }

            // 3. SEND SINGLE INTEGER RESULT BACK to Producer
            if (xQueueSend(result_queue,(void*) &xStep, pdMS_TO_TICKS(100)) != pdPASS) {
                ESP_LOGE(TAG, "Consumer: Failed to send result back to Producer. Result: %d", calculated_result);
            } else {
                ESP_LOGI(TAG, "Consumer: Successfully sent result %d back to Producer.", calculated_result);
            }

        } else {
            ESP_LOGE(TAG, "Consumer: Error receiving data from queue.");
        }
    }
}

    
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
  // Queue 1 (Producer -> Consumer): Now holds items of size DataPacket_t.
    calculation_queue = xQueueCreate(5, sizeof(DataPacket_t));
    
    // Queue 2 (Consumer -> Producer): Holds 1 item of size int (only needs to hold one result).
    result_queue = xQueueCreate(1, sizeof(int));

    if (calculation_queue == NULL || result_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create one or both FreeRTOS Queues. Cannot proceed.");
        return;
    }

    // 2. Create the Consumer Task (the other thread)
    // Pin it to Core 1 (APP_CPU_NUM)
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        performAcuation,      // Function that implements the task
        "performAcuation",     // Text name for the task
        4096,               // Stack size (in bytes)
        NULL,               // Parameter passed to the task 
        5,                  // Priority
        NULL,               // Task handle
        APP_CPU_NUM         // Core ID (Core 1)
    );

    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Consumer Task.");
        return;
    }
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

  digitalWrite(EnPin, LOW); //auto by default
}

void loop() {
  //https://www.mouser.com/ds/2/408/TB6600HG-483084.pdf
  //500millisecond setup time, and another 500 millisecond hold time

  //for angles, code can be as follows
  currentCLKCount = encoder.getCount();
  angle = ((double)currentCLKCount) * (360.0 / (double)ENCODER_PPR);

  Serial.print("Angle: ");
  Serial.println(angle);
  long long time = get_time();
  angles.push_back(angle);
  times.push_back(time);
  double dError = 0.0;
  double prevError = 0.0;
  if (angles.size() >= 2 && times.size() >= 2) {
    angular_velocity = (angles[angles.size() - 1]-angles[angles.size() - 2])/(double)((times[angles.size() - 1]-times[angles.size() - 2]));
    angles.erase(angles.begin());
    times.erase(times.begin()); //to keep memory small
  }
  else {
    angular_velocity = 0;
  }
  //for x position and x', refer to time
  if (times.size() >= 2) {
    velocity = (xStep - prev_x)/(times[times.size() - 1] - times[times.size() - 2]); //steps/ms
    dError = prevError;
    //TODO Change this to properly calculatte dError
  }
  //angular velocity calculated by luke: vector in C++; check github
  /*For the control/math itself*/
  //most likely need variable x, x', θ, θ'
  //can be found with lagrange equations or other method

  //psuedocode:
  //double error = θ - 90;
  double error = (angle - targetAngle);
  if (times.size() >= 2) {
    velocity = (xStep - prev_x)/(times[times.size() - 1] - times[times.size() - 2]); //steps/ms
    dError = (error - prevError)/(times[times.size() - 1] - times[times.size() - 2]); 
  }
  //if error is less than 0, then angle is to the right of 90 degrees
  //right now, error is in angles, need to consider angular velocity, cart velocity, position, and angle (controls)
  //track is only 0.5m long, so there has to be a limit on speed, current value is arbitrary
  const double maxSpeed = 500.0;

  //no integral term
  double PD = -(Kp * error + Kd * dError); //output from controller in terms of degrees

  //another arbitrary value, approximation of steps per degree
  //Don't think it will be accurate because of acceleration, as 5 steps to the right when the pendulum is moving down is different from 5 steps to the right when the pendulum is on its way up
  double stepsToAngle = 4.0;

  double driverInput = PD * stepsToAngle; //desired angle change * step per angle
    if (driverInput < 0) {
      digitalWrite(dirPin, LOW);
      driverInput = -driverInput;
    } else {
      digitalWrite(dirPin, HIGH);
    }

    if (xQueueSend(calculation_queue, &error, pdMS_TO_TICKS(1000)) != pdPASS) {
            ESP_LOGE(TAG, "Producer: Failed to send data packet to the consumer queue.");
    }
    if (xQueueReceive(result_queue,(void*) &xStep, pdMS_TO_TICKS(100)) == pdPASS) {
                prev_x=xStep;
            }        
}