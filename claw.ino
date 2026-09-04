#include <Servo.h>

Servo myServo;

//  Pins 
const int servoPin = 8;
const int trigPin  = 11;
const int echoPin  = 10;

//  Servo positions 
const int posOpen  = 180;
const int posClose = 0;

//  Thresholds (cm) 
// Spread wider than sonar jitter to avoid state chatter near boundaries.
const float GRAB_THRESHOLD    = 10.0;  // object close enough to attempt grab
const float LIFT_CONFIRM      = 18.0;  // distance proving the object was lifted
const float LOST_THRESHOLD    = 25.0;  // object gone / dropped: reopen and reset

//  Timing (ms) 
const unsigned long HOLD_TIME   = 300;   // object must stay in range before grabbing
const unsigned long LIFT_WAIT   = 1200;  // window to confirm a successful lift
const unsigned long LOOP_PERIOD = 50;    // fixed sample interval
const unsigned long ECHO_TIMEOUT = 25000; // us; caps pulseIn blocking

#define AVG_SIZE 5
float distances[AVG_SIZE];
int idx = 0;
bool bufferFilled = false;

//  State machine 
enum ClawState { SEARCHING, SETTLING, GRABBING, HOLDING };
ClawState state = SEARCHING;

unsigned long settleStart = 0;
unsigned long grabStart   = 0;
unsigned long lastLoop    = 0;

// Rolling average. Returns false until the buffer has real data.
bool getAverageDistance(float newVal, float &out) {
  distances[idx] = newVal;
  idx = (idx + 1) % AVG_SIZE;
  if (idx == 0) bufferFilled = true;
  if (!bufferFilled) return false;

  float sum = 0;
  for (int i = 0; i < AVG_SIZE; i++) sum += distances[i];
  out = sum / AVG_SIZE;
  return true;
}

// Returns distance in cm, or -1 on timeout (no echo).
float readSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}


void openClaw()  { myServo.write(posOpen);  }
void closeClaw() { myServo.write(posClose); }


void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);
  openClaw();

  Serial.begin(9600);
  lastLoop = millis();
}


void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_PERIOD) return;  // fixed-rate sampling
  lastLoop = now;

  float raw = readSonar();
  if (raw < 0) return;  // no echo this cycle, skip

  float distance;
  if (!getAverageDistance(raw, distance)) return;  // wait for buffer to fill

  Serial.print("State: ");
  Serial.print(state);
  Serial.print("  Dist: ");
  Serial.println(distance);

  switch (state) {

    case SEARCHING:
      // Wait for an object to enter grab range.
      if (distance <= GRAB_THRESHOLD) {
        state = SETTLING;
        settleStart = now;
      }
      break;

    case SETTLING:
      // Confirm the object is stable in range before committing.
      if (distance > GRAB_THRESHOLD) {
        state = SEARCHING;              // drifted out, abort
      } else if (now - settleStart >= HOLD_TIME) {
        Serial.println("Closing claw");
        closeClaw();
        state = GRABBING;
        grabStart = now;
      }
      break;

    case GRABBING:
      // Closed-loop check: did the grip actually lift the object?
      if (distance >= LIFT_CONFIRM) {
        Serial.println("Lift confirmed");
        state = HOLDING;                // success
      } else if (now - grabStart >= LIFT_WAIT) {
        Serial.println("Grab failed, retrying");
        openClaw();
        state = SEARCHING;              // failure, reopen and retry
      }
      break;

    case HOLDING:
      // Hold until the object is released or lost.
      if (distance >= LOST_THRESHOLD) {
        Serial.println("Object released, reopening");
        openClaw();
        state = SEARCHING;
      }
      break;
  }
}
