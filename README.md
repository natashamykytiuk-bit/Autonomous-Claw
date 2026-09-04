# autonomous-claw

An Arduino Uno reads distance from an ultrasonic sensor, drives a 180° servo to open/close the claw, and runs a four state machine.

This design is a closed loop, it does not assume that a grab worked, instead it confirms the lift from sensor feedback and self corrects on failure.
