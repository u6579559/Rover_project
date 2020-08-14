#include <Servo.h>

// Set up definitions for each pin
#define MTR1_FWD_PIN 4
#define MTR1_BCK_PIN 5
#define MTR2_FWD_PIN 7
#define MTR2_BCK_PIN 6
#define SONAR_IN_PIN 11
#define SONAR_OUT_PIN 10

// Constants (fine tune these as needed)
#define SERVO_MOVE_TIME 350           // The time in miliseconds we should wait for the servo to move before progressing
#define TURN_TIME 1870                // The time in miliseconds we should turn for the rover to turn 90 degrees
#define SERVO_FORWARDS_ANGLE 30      // The angle to write to the servo to make it point forwards
#define SERVO_DIAG_ANGLE 75
#define SERVO_LEFT_ANGLE 120           // The angle to write to the servo to make it point left
#define MINIMUM_DISTANCE_CLEAR 13.0   // The minimum distance in cm we should see when looking for a clear route
#define FORWARDS_STOP_DISTANCE  4.0    // The distance to the wall in front of us when we should stop
#define FORWARDS_START_CHECKING_DISTANCE 7.0 // The distance from the front wall where we just keep checking forwards to make sure we don't crash
#define TURN_LEFT_DELAY 1700         // After we detect a passage to the left we should wait this many milisecods so we're aligned and don't clip the corner
#define TURN_RIGHT_DELAY 100          // NOT USED
#define FORWARDS_AFTER_LEFT_TURN_TIME 700 // After we make a left tuurn, just go forwards for this long before resuming the navigate loop
#define DISTANCE_TO_WALL 4.0          // The distance we will try to remain next to the left wall
#define DISTANCE_TO_WALL_TURN_TIME 160 // The time we turn for when we are not aligned with wall
#define REALIGN_EVERY 0               // How many iterations of navigate() we realign with the left wall yu know what i mean

Servo servo;
int servoPosition = 1;  // If true, the ultrasound is facing forwards. If false, facing left.
char data = 0;
bool autonomous = false; // If true, robot is operating in autirnonmousa mode
int realignmentLoop = 0;
bool alignmentFunctionEnabled = true;
bool atAngle = false;


void setup() {
  // Set appropriate pin modes:
  pinMode(MTR1_FWD_PIN, OUTPUT);
  pinMode(MTR1_BCK_PIN, OUTPUT);
  pinMode(MTR2_FWD_PIN, OUTPUT);
  pinMode(MTR2_BCK_PIN, OUTPUT);
  pinMode(SONAR_OUT_PIN, OUTPUT);
  pinMode(SONAR_IN_PIN, INPUT);
  servo.attach(13);
  servo.write(SERVO_FORWARDS_ANGLE);

  // Start the bluetooth serial connection
  Serial.begin(9600);
}


void loop() {
  
  if(Serial.available() > 0){
    data = Serial.read();
    
    if(data == 'D'){ // auto mode on or off
      autonomous = true;
    }
    if(data == 'C'){
      autonomous = false;
      servo.write(SERVO_FORWARDS_ANGLE);
    }
    
  } //end of serial available scope


  if (autonomous){
    navigate();
  }else{
    manual();
  }
  
}

void manual(){ // handles manual control behaviour
  
  if (data == '0'){
    stopMotors();
  }else if (data == '8'){
    float forwardsDistance = getForwardsDistance();
    if (forwardsDistance > FORWARDS_STOP_DISTANCE and forwardsDistance != 0.0){
      forwards();
    }else{
      stopMotors();
    }
  }else if (data == '2'){
    backwards();
  }else if (data == '4'){
    turn(true);
  }else if (data == '6'){
    turn(false);
  }else if (data == 'E'){
    alignmentFunctionEnabled = !alignmentFunctionEnabled;
  }
}

void navigate(){  // Autonomously navigates the maze. This function should be called as quickly as possible
  
  
  delay(500);
  // getForwardDistance is called first because we don't want to run into a wall straight away after turning right.
  stopMotors();
  float forwardsDistTemp = getForwardsDistance();
  float diagDistTemp = getDiagDistance();
  float leftDistance = getLeftDistance();
  forwards();
  
  
  if (forwardsDistTemp < FORWARDS_STOP_DISTANCE and forwardsDistTemp != 0.0 and !atAngle){  // Reached the end of the passage, or detecting nothing (mayb on ramp?)
    turnRight();  // The getForwardsDistance in the next iteration should take care of if we are at the end of a dead end.
    //realignmentLoop = -3;   // Don't check alignemnts until we are well around the corner.
  }else if (forwardsDistTemp < FORWARDS_STOP_DISTANCE and forwardsDistTemp != 0.0 and atAngle){
    turnRight();
  }else if (diagDistTemp < 3.0 and !atAngle){
    atAngle = true;
    turn(false);
    delay(TURN_TIME/2);
    forwards();
    delay(4000);
  }else if (diagDistTemp > 15.0 and atAngle){
    atAngle = false;
    turn(true);
    delay(TURN_TIME/1.5);
  }else if (diagDistTemp < 3.0 and atAngle) {
    atAngle = false;
    turn(false);
    delay(TURN_TIME/1.5);
  }else{
    if (leftDistance > MINIMUM_DISTANCE_CLEAR){  // Found a passage to the left!
      delay(TURN_LEFT_DELAY); // We might want to wait a bit after we've detected a passage to the left, to make sure we're aligned.
      turnLeft();
      forwards();
      delay(FORWARDS_AFTER_LEFT_TURN_TIME); // Make sure we are around the corner
      //realignmentLoop = -3;   // Don't check alignemnts until we are well around the corner.
    }else{  // Nothing changed, smooth sailing

      // Alignments, only is called if we are going down a straight path
      if (alignmentFunctionEnabled){
        realignmentLoop ++;
        // Make sure we are still aligned by maintaining constant distance to left wall. This might be bad in a large open space.
        if (realignmentLoop > REALIGN_EVERY){ // Only align every REALIGN_EVERY iterations, to avoid over steering
          realignmentLoop = 0;
          if (leftDistance > DISTANCE_TO_WALL + 0.5){ // If we are too far from the wall, turn slightly left
            turn(true);
            delay(DISTANCE_TO_WALL_TURN_TIME);
            forwards();
          }else if (leftDistance < DISTANCE_TO_WALL - 0.5){ // Too close to the wall, turn slightly right
            turn(false);
            delay(DISTANCE_TO_WALL_TURN_TIME);
            forwards();
          }
        }
      }// eND OF ALIGNMENTS
      
    }
  }

  // This last bit keeps the rover checking forwards if we are kind of close to the wall.
  /*forwards();
  while (forwardsDistTemp < FORWARDS_START_CHECKING_DISTANCE and forwardsDistTemp > FORWARDS_STOP_DISTANCE){ // If we are getting kind of close to the front wall, stop checking left and just check forwards
    forwardsDistTemp = getForwardsDistance();  // Constantly check forwards
    delay(50);
  }         // While we are kind of, but not too close, to the forwards wall, rover is stuck in this loop until we reach the forwards wall.
  */
}

void turnLeft(){  // Turns left ~90 degrees
  turn(true);
  delay(TURN_TIME);
  stopMotors();
}

void turnRight(){ // Turns right ~90 degrees
  turn(false);
  delay(TURN_TIME);
  stopMotors();
}


//// ULTRASOUND FUNCTIONS ////

float getForwardsDistance(){  // Returns the distance to the front of the robot
  if (servoPosition != 1){  // If the servo is facing left at the moment, rotate it to forwards
    servo.write(SERVO_FORWARDS_ANGLE);
    servoPosition = 1;
  }
  delay(SERVO_MOVE_TIME); // wait for the servo to move
  return getDistance();   // make the measurement
}

float getDiagDistance(){  // Returns the distance to the front of the robot
  if (servoPosition != 2){  // If the servo is facing left at the moment, rotate it to forwards
    servo.write(SERVO_DIAG_ANGLE);
    servoPosition = 2;
  }
  delay(SERVO_MOVE_TIME); // wait for the servo to move
  return getDistance();   // make the measurement
}

float getLeftDistance(){  // Returns the distance to the left of the robot
  if (servoPosition != 3){  // If the servo is facing left at the moment, rotate it to forwards
    servo.write(SERVO_LEFT_ANGLE);
    servoPosition = 3;
  }
  delay(SERVO_MOVE_TIME); // wait for the servo to move
  return getDistance();   // make the measurement
}

float getDistance(){  // returns ultrasound distance in centimetres from 5 trials
  float average = 0;        // the average of the 5 pings, including outliers
  float highestError = 0.0; // the error of the ping furthest from the average
  int highestErrorPosition = 0; //position of this ping in the array
  int distances[5];
  
  for(int i = 0; i < 5; i++) {      // Get 5 different ping distances
      digitalWrite(SONAR_OUT_PIN, HIGH);
      delayMicroseconds(20);
      digitalWrite(SONAR_OUT_PIN, LOW);
      int d = pulseIn(SONAR_IN_PIN, HIGH, 12000)*0.00005*343.0; // Get ping, give up if ping > 20m
      distances[i] = d;
      delay(20); // We may need a delay before getting another reading???
  }
  
  for (int i=0;i<5;i++){        // calculate the average of all the distances
      average += distances[i];
  }
  average = average / 5;
  
  for (int i=0;i<5;i++){        // find the position of the ping with the worst error
      if (abs(average-distances[i]) > highestError){
          highestError = abs(average-distances[i]);
          highestErrorPosition = i;
      }
  }
  
  average = 0.0;
  int newDistances[4];
  int n=0;
  for (int i=0;i<5;i++){    // put the new distances into a new array
    if (i != highestErrorPosition){
      newDistances[n] = distances[i];
      n ++;
    }
  }
  // Repeat the above process but for array of 4. I'm so sorry....

  for (int i=0;i<4;i++){        // calculate the average of all the distances
      average += newDistances[i];
  }
  average = average / 4;

  highestError = 0.0; // the error of the ping furthest from the average
  highestErrorPosition = 0; //position of this ping in the array
  for (int i=0;i<4;i++){        // find the position of the ping with the worst error
      if (abs(average-newDistances[i]) > highestError){
          highestError = abs(average-newDistances[i]);
          highestErrorPosition = i;
      }
  }

  average = 0.0;
  for (int i=0;i<3;i++){
    if (i != highestErrorPosition){
      average += newDistances[i];
    }
  }
  
  average = average / 3;
  Serial.println(average);
  return average;
}


//// MOTOR FUNCTIONS ////

void forwards(){
  digitalWrite(MTR1_FWD_PIN, HIGH);
  digitalWrite(MTR2_FWD_PIN, HIGH);
  digitalWrite(MTR1_BCK_PIN, LOW);
  digitalWrite(MTR2_BCK_PIN, LOW);
}

void stopMotors(){
  digitalWrite(MTR1_FWD_PIN, LOW);
  digitalWrite(MTR2_FWD_PIN, LOW);
  digitalWrite(MTR1_BCK_PIN, LOW);
  digitalWrite(MTR2_BCK_PIN, LOW);
}

void backwards(){
  digitalWrite(MTR1_FWD_PIN, LOW);
  digitalWrite(MTR2_FWD_PIN, LOW);
  digitalWrite(MTR1_BCK_PIN, HIGH);
  digitalWrite(MTR2_BCK_PIN, HIGH);
}

void turn(bool left){
  if (left){
    digitalWrite(MTR1_FWD_PIN, LOW);
    digitalWrite(MTR2_FWD_PIN, HIGH);
    digitalWrite(MTR1_BCK_PIN, HIGH);
    digitalWrite(MTR2_BCK_PIN, LOW);
  }else{
    digitalWrite(MTR1_FWD_PIN, HIGH);
    digitalWrite(MTR2_FWD_PIN, LOW);
    digitalWrite(MTR1_BCK_PIN, LOW);
    digitalWrite(MTR2_BCK_PIN, HIGH);
  }
}

