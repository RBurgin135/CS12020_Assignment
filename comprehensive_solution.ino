//========== DECLARATION ==========
#include <EEPROM.h>
//SERVOS ==========
#include <Servo.h>
Servo servoLeft; const int LEFTSTOP = EEPROM[0]; const int LEFT_SERVO_PIN = 6; const int LEFTOFFSET = EEPROM[1];
Servo servoRight; const int RIGHTSTOP = EEPROM[2]; const int RIGHT_SERVO_PIN = 5; const int RIGHTOFFSET = EEPROM[3];
const int MS_PER_CM = 182;
const int MS_PER_DEG = 30;

//LDR ==========
struct LDR_master {
  const int pin;
  int mid;
  int reading;
  bool judgement; //true if dark, false if light
};
int LDR_init_mid[3] = {(EEPROM[4] + EEPROM[6] + (EEPROM[5] + EEPROM[7]) * 256) / 2,
                     (EEPROM[8] + EEPROM[10] + (EEPROM[9] + EEPROM[11]) * 256) / 2,
                     (EEPROM[12] + EEPROM[14] + (EEPROM[13] + EEPROM[15]) * 256) / 2};
LDR_master a = {.pin = A2, .mid = LDR_init_mid[0]};
LDR_master b = {.pin = A1, .mid = LDR_init_mid[1]};
LDR_master c = {.pin = A0, .mid = LDR_init_mid[2]};
LDR_master LDRs[3] = {a, b, c};

//LED ==========
const int GREEN = 7;
const int YELLOW = 12;
const int RED = 13;

//IR ==========
const int IR = 3; const int IR_rate = 38000;
const int SENSOR = 2;

//BARCODE ==========
bool onBlack = false;
bool resultFound = false;
int whiteMeasure[2] = {0,0};
int whiteIndex = -1;




//========== FUNCTIONS ==========
//SERVOS ==========
void setSpeed (int left, int right) {
  /* outputs a speed to each servo
     after processing the value */
  int outleft = LEFTSTOP + left;
  int outright = RIGHTSTOP - right;
  servoLeft.write(outleft);
  servoRight.write(outright);
}

void turn (int deg) {
  /*rotate the robot deg, degrees clockwise if deg is
    positive rotate anti-clockwise if deg is negative */
  clearLEDs();
  setLEDs(GREEN, 1);
  if (deg > 0)  setSpeed(10, -10);
  else          setSpeed(-10, 10);
  delay(MS_PER_DEG * abs(deg));
  recover();
}

void move (int cm) {
  /*move the robot n cm's forwards if cm is positive
    move the robot backwards if cm is negative */
  clearLEDs();
  setLEDs(YELLOW, 1);
  if (cm > 0)   setSpeed(10, 10);
  else          setSpeed(-10, -10);
  delay(MS_PER_CM * abs(cm));
  recover();
}

void stop () {
  /* make the robot come to a complete stop */
  setSpeed(0, 0);
}

void recover () {
  /* prevent the movement commands
     from effecting each other */
  stop();
  delay(500);
}


//LDR ==========
bool findLDRjudgement (LDR_master LDR) {
  /* find whether the LDR is on
     black (true) from its reading */
  return LDR.reading < LDR.mid;
}

void readAllLDRs () {
  /* find all the readings and then their judgements, 
     whilst disregarding anomalous results*/
  for (int l = 0; l < 3; l++) {
    LDR_master *x = &LDRs[l];
    bool judgements[5];
    for (int r = 0; r < 5; r++) {
      (*x).reading = analogRead((*x).pin);
      judgements[r] = findLDRjudgement(*x);
      delay(10);
    }

    //remove anomalous
    (*x).judgement = deduceJudgement(judgements);
  }
}

bool deduceJudgement (bool j[5]) {
  /* given a list of 5 LDR judgements, this will find the mode 
     of the judgements, used in removing anomalous results */
  int total = 0;
  for (int i=0; i<5; i++) {
    total += j[i];
  }
  return total >= 3;
}

bool eitherBlackQuery () {
  /* find whether either far-left or
    far-right LDR's are on black (true) */
  return LDRs[0].judgement || LDRs[2].judgement;
}

bool bothBlackQuery (){
  /* find whether both the far-left 
    and far-right LDR's are on black(true) */
  return LDRs[0].judgement && LDRs[2].judgement;
}

bool bothWhiteQuery (){
  /* find whether both the far-left 
    and far-right LDR's are on white(false) */
  return !LDRs[0].judgement && !LDRs[2].judgement;
}


//LED ==========
void setLEDs (int LED, int state) {
  /* sets the specified LED to the
     specified state */
  digitalWrite(LED, state);
}

void clearLEDs () {
  /* set all LED's to off(LOW) */
  digitalWrite(GREEN, 0);
  digitalWrite(YELLOW, 0);
  digitalWrite(RED, 0);
}


//FOLLOW LINE ==========
void followLine () {
  /* if both far-left and far-right LDR's are not both on black,
     then it will examine if the far-left LDR is on black, if so
     the left servo will halt thus turning the robot right. Then it
     does the opposite to the far-right LDR */
  int leftServoPower = 10; int rightServoPower = 10;
  if (!bothBlackQuery()) {
      if (LDRs[0].judgement)  leftServoPower = 0;
      if (LDRs[2].judgement)  rightServoPower = 0;
  }
  setSpeed(leftServoPower, rightServoPower);
}


// BARCODE ==========
void readBarcode () {
  /* main routine responsible for reading a barcode: manages what 
    happens if there is a transition or whether to measure white 
    or move through the black bars */
  clearLEDs();
  resetBarcode();
  while (!resultFound){
    readAllLDRs();
    if(transitionQuery())   transition();
    if(onBlack)             moveThroughBlack();
    else                    measureWhite();
  }
  while (eitherBlackQuery()) {
    readAllLDRs();
    moveThroughBlack();
  }
}

void resetBarcode() {
  /* resets all the variables relating to reading 
    a barcode */
  onBlack = false;
  resultFound = false;
  whiteMeasure[0] = 0;
  whiteMeasure[1] = 0;
  whiteIndex = -1;
}

bool transitionQuery () {
  /* checks to see if the robot has moved from white 
    to black or vice-versa, to see if it has reached 
      the end of the stripe */
  if (onBlack)    return bothWhiteQuery();
  else            return bothBlackQuery();
}

void transition () {
  /* reverts onBlack and executes deduceWhiteMeasure() */
  if (!onBlack)   deduceWhiteMeasure();
  onBlack = !onBlack;
}

void measureWhite () {
  /* adds to the measurement saved in whiteMeasure */
  move(1);
  whiteMeasure[whiteIndex]++;
}

void moveThroughBlack () {
  /* moves through black bars */
  int leftServoPower = 10; int rightServoPower = 10;
  setSpeed(leftServoPower, rightServoPower);
}

void deduceWhiteMeasure () {
  /* deduces what the measurement that was taken means */
  clearLEDs();
  if (whiteMeasure[0] > 8){
    setLEDs(GREEN,1); //turn right
    resultFound = true;
    nudgeLDRMids(50); //fainter line

  } else if (whiteMeasure[0] > 0 && whiteMeasure[1] > 0){
    setLEDs(RED,1); //turn left
    resultFound = true;
    nudgeLDRMids(-90); //darker background
  } 
  whiteIndex ++;
}

void nudgeLDRMids (int nudge) {
  /* adjusts each LDR's mid value to respond 
    to the barcodes command */
  for (int i=0; i<3; i++) {
    LDRs[i].mid = LDR_init_mid[i] + nudge;
  }
}


//OBSTACLE ==========
void ObstacleAvoidance () {
  /* main loop for maneuvering around an obstacle, 
    will move around the right-hand-side of the object */
  turn(70);
  move(30);
  turn(-70);
  move(20);
  turn(-70);
  while (!refindLineQuery()){
    readAllLDRs();
    move(3);
  }
}

bool scanForObstacle () {
  /* sends a pulse from the IR to the sensor, return the reading */
  tone(IR, IR_rate);
  delay(250);
  bool reading = digitalRead(SENSOR) == LOW;
  noTone(IR);
  return reading;
}

bool refindLineQuery () {
  /* find if the robot has refound the line 
    once it has navigated around the obstacle */
    return !LDRs[0].judgement && LDRs[2].judgement;
}


//========== MAIN ==========
void setup() {
  Serial.begin(9600);

  //SERVOS
  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);

  //LDR
  for (int i = 0; i < 3; i++) {
    pinMode(LDRs[i].pin, INPUT);
  }

  //LED
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);

  //IR
  pinMode(IR, OUTPUT);
  pinMode(SENSOR, INPUT);
}

void loop() {
  if (scanForObstacle())    ObstacleAvoidance();
  readAllLDRs();
  if (bothBlackQuery())    readBarcode();
  followLine();
}
