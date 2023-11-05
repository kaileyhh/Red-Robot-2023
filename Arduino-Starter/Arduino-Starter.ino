#include "constants.h"

// Replace 12345 with the correct team number and then uncomment the line below.
#define TEAM_NUMBER 8

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 20 < TEAM_NUMBER
#error "Team number must be within 1 and 20"
#endif

bool isServoIntake; //for state of servo intake
bool isServoScore; // for state of servo score

bool prevRB; //to see if RB pressed (not held down)
bool prevLB; //to see if LB pressed (not held down)

float lastVal;
float lastError;

float kP = 0.00025; // 0.00025; // 0.00025
float kD = 0.00003; // 0.00007

bool isAuto;
bool prevA;

bool scoredPieceInAuto;
// float kD = 0.0;

float currServo1Angle = DEFAULT_SERVO_ANGLE_1;
float currServo4Angle = DEFAULT_SERVO_ANGLE_4;

int sensors[6];
int start_time;


float clamp(float in, float min, float max) {
  if (in > max) {
    Serial.write("clamp exceed max\n");
    return max;
  }
  if (in < min) {
    Serial.write("clamp exceed min\n");
    return min;
  }
  return in;
}

int clamp_int(int in, int min, int max) {
  if (in > max) {
    Serial.write("clamp exceed max\n");
    return max;
  }
  if (in < min) {
    Serial.write("clamp exceed min\n");
    return min;
  }
  return in;
}

void setServo(int angle1, int angle4) {
  // Serial.print("setting servo: ");

  angle1 = clamp_int(angle1, 0, 180);
  angle4 = clamp_int(angle4, 0, 180);

  // Serial.print(angle1);
  // Serial.print(", ");
  // Serial.print(angle4);
  // Serial.print("\n");

  RR_setServo1(angle1);
  RR_setServo4(angle4);
}

void setGlobalServoAngles(int angle1, int angle4) {
  currServo1Angle = angle1;
  currServo4Angle = angle4;
}

void setServosToGlobal() {
  setServo(currServo1Angle, currServo4Angle);
}

void setServoIntake() {
  setGlobalServoAngles(INTAKE_SERVO_ANGLE_1, INTAKE_SERVO_ANGLE_4);
  setServo(INTAKE_SERVO_ANGLE_1, INTAKE_SERVO_ANGLE_4);
}

void setServoScore() {
  setGlobalServoAngles(SCORE_SERVO_ANGLE_1, SCORE_SERVO_ANGLE_4);
  setServo(SCORE_SERVO_ANGLE_1, SCORE_SERVO_ANGLE_4);
}

void setServoDefault() {
  setGlobalServoAngles(DEFAULT_SERVO_ANGLE_1, DEFAULT_SERVO_ANGLE_4);
  setServo(DEFAULT_SERVO_ANGLE_1, DEFAULT_SERVO_ANGLE_4);
}

float getUltrasonic() {
  return RR_getUltrasonic();
}

bool ultrasonicSeesObject() {
  float ult = getUltrasonic();
  return ult < ULTRASONIC_SENSE_DISTANCE && ult > 0.0001;
}

void stopDrivetrain() {
  RR_setMotor1(0.0);
  RR_setMotor2(0.0);
}

void arcadeDrive(float thrust, float rotation) {
  // rotation = clamp(rotation, -0.5, 0.5);
  thrust = abs(thrust) > 0.001 ? thrust : 0;
  rotation = abs(rotation) > 0.001 ? rotation : 0;

  // thrust = thrust * thrust * (thrust < 0.0 ? -1.0 : 1.0);
  // rotation = rotation * rotation * (rotation < 0.0 ? -1.0 : 1.0);
  // Serial.print("driving power: ");
  // Serial.print(clamp(thrust + rotation, -1.0, 1.0));
  // Serial.print(" , ");
  // Serial.print(clamp(thrust - rotation, -1.0, 1.0));
  // Serial.print("\n");
  RR_setMotor1(clamp(thrust + rotation, -1.0, 1.0));
  RR_setMotor2(clamp(thrust - rotation, -1.0, 1.0));
}

void debugPrints(bool btnA, bool btnB, bool btnX, bool btnY) {
  Serial.print("Ultrasonic=");
  Serial.print(RR_getUltrasonic());
  Serial.print("\n");
  // Serial.print(" ;; ");
  // int sensors[6];

  // Serial.print("LINE SENSORS = ");
  // RR_getLineSensors(sensors);
  // for (int i = 0; i < 6; i++) {
  //   Serial.print(sensors[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("\n");
  // delay(1000);


  //follows line
  // line_follow(sensors);

  // Serial.print(btnA ? 1 : 0);
  // Serial.print(btnB ? 1 : 0);
  // Serial.print(btnX ? 1 : 0);
  // Serial.print(btnY ? 1 : 0);
  // Serial.println();
}

/* --- AUTO COMMANDS --- */

void scorePieceAuto() {
  setServoScore();
  delay(SCORE_TIME);
  setServoDefault();
}

void drivePastYellowAuto(int sensors[], int start_time) {
  if (millis() - start_time < 8000) {
    line_follow_pid(sensors);
  } else {
    isAuto = false;
  }
}

void driveScoreAuto(int sensors[], int start_time) {
  float currTime = millis();
  bool ult = false;
  if (((int)currTime % 250) < 5.0 && currTime - start_time > 12000) {
   ult = ultrasonicSeesObject();
  }
  if (!scoredPieceInAuto && !ult) {
    line_follow_pid(sensors);
  } else if (!scoredPieceInAuto) {
    Serial.print("SEE TARGET, SCORRE NOW\n");
    scorePieceAuto();
    scoredPieceInAuto = true;
    isAuto = false;
  }
}

void driveScoreAutoIndependent(int sensors[], int start_time) {
  bool ult = false;
  
  if (isAuto) {
    while (!ult) {
      float currTime = millis();

      RR_getLineSensors(sensors);
      for (int i = 0; i < 6; i++) {
        Serial.print(sensors[i]);
        Serial.print(" ");
      }
      Serial.print("\n");

      line_follow_pid(sensors);
      ult = ultrasonicSeesObject();
      delay(20);
    } 
    stopDrivetrain();

    Serial.print("SEE TARGET, SCORRE NOW\n");
    scorePieceAuto();
    scoredPieceInAuto = true;
    isAuto = false;
  }
  
}

void move_forward() {
  RR_setMotor1(DRIVE_FORWARD_POWER);
  RR_setMotor2(DRIVE_FORWARD_POWER);
}

void move_left() {
  RR_setMotor1(TURN_BACKWARD_POWER);
  RR_setMotor2(TURN_FORWARD_POWER);
}

void move_right() {
  RR_setMotor1(TURN_FORWARD_POWER);
  RR_setMotor2(TURN_BACKWARD_POWER);
}

/* --- END AUTO COMMANDS --- */

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // set all modes to default (nothing pressed/held)
  // start in default position
  setServoDefault();
  isServoIntake = false;
  isServoScore = false;
  prevRB = false;
  prevLB = false;
  lastVal = 2500.0;
  lastError = 0.0;

  prevA = false;
  isAuto = false;
  start_time = millis();
  scoredPieceInAuto = false;
}

int temp = 0;

void loop() {
  // Read the four joystick axes
  // These will be in the range [-1.0, 1.0]
  float rightX = RR_axisRX();
  float rightY = RR_axisRY();
  float leftX  = RR_axisLX();
  float leftY  = RR_axisLY();

  arcadeDrive(leftY, rightX);

  // Get the button states
  bool btnA = RR_buttonA();
  bool btnB = RR_buttonB();
  bool btnX = RR_buttonX();
  bool btnY = RR_buttonY();
  bool btnRB = RR_buttonRB();
  bool btnLB = RR_buttonLB();


  // ------------------ SET AUTO ------------------
  if (btnA) {
    if (btnA != prevA) {
      Serial.print("A changed");
      start_time = millis();
      isAuto = true;
    }
    
    driveScoreAutoIndependent(sensors, start_time);
  } else {
    isAuto = false;
  }

  // debugPrints(btnA, btnB, btnX, btnY);
  
  // if (isAuto) {
  //   // debugPrints(btnA, btnB, btnX, btnY);
  //   // line_follow_pid(sensors);
  //   // driveScoreAuto(sensors, start_time);
  //   // delay(10);
  // } else {
  if (!isAuto) {

    // left bumper - intake
    if (btnLB && btnLB != prevLB) { //if the button is being pressed and it is just pressed
      if (isServoIntake) { //if the servo is in intake position
        setServoDefault(); // set servo to default
      } else {  //servo is in default position
        setServoIntake(); //intake
      }
      isServoIntake = !isServoIntake; //set servo intake boolean to reverse it later
      isServoScore = false; //score is false so if in score position, then if pressed score, will not set to default
    }
    // right bumper - score
    if (btnRB && btnRB != prevRB) { //same logic for score
      if (isServoScore) {
        setServoDefault();
      } else {
        setServoScore();
      }
      isServoScore = !isServoScore;
      isServoIntake = false;
    } 
    

    setServosToGlobal();
    delay(20);
  }
  

  // This is important - it sleeps for 0.02 seconds (= 50 times / second)
  // Running the code too fast will overwhelm the microcontroller and peripherals

  prevA = btnA;
  prevLB = btnLB;
  prevRB = btnRB;
}

float process_sensors(int sensors[]) {
  //assumes that black line is a solid continuous line

  bool on_line = false;
  float avg = 0.0;
  float sum = 0.0;

  for (int i = 0; i < 6; i++) {
    float val = sensors[i] / 1000.0;
    if (val > (LINE_VAL / 1000.0)) {
      on_line = true;
    }
    avg += val * (i * 1000);
    sum += val;
  }

  if (!on_line) {
    return lastVal < 2500 ? 5000.0 : 0.0;
  }
  
  lastVal = avg / sum;
  // Serial.print("sensor processed: ");
  // Serial.print(lastVal);
  // Serial.print("\n");
  return lastVal;
}

void line_follow_pid(int sensors[]) {
  RR_getLineSensors(sensors);

  float sensorVal = process_sensors(sensors);
  float error = LINE_SETPOINT - sensorVal;
  // error > 0 if too far left

  if (error > LINE_TOLERANCE) {
    Serial.print("GO RIGHT\n");
  } else if (error < -LINE_TOLERANCE) {
    Serial.print("GO LEFT\n");
  } else {
    Serial.print("We r ok\n");
  }

  Serial.print("error: ");
  Serial.print(error);

  float adjust = error * kP + kD * (error - lastError);
  // adjust really big if error is really big, 

  adjust = abs(error) < LINE_TOLERANCE ? 0 : adjust;

  lastError = error;

  Serial.print(", motor 1 power: ");
  Serial.print(DRIVE_FORWARD_POWER + adjust);
  Serial.print(", motor 2 power:");
  Serial.print(DRIVE_FORWARD_POWER - adjust);
  Serial.print("\n");

  RR_setMotor1(clamp(DRIVE_FORWARD_POWER + adjust, -1.0, 1.0));
  RR_setMotor2(clamp(DRIVE_FORWARD_POWER - adjust, -1.0, 1.0));
}

// void line_follow() {
//   int sensors[6];
//   RR_getLineSensors(sensors);
//   //precondition: sensor is set such that the black line is in between sensors
//   int sr = sensors[0] + sensors[1] + sensors[2];
//   int sl = sensors[5] + sensors[4] + sensors[3];

//   float lr_ratio = sl/sr;

//   if (lr_ratio > 0.8 && lr_ratio < 1.25) {
//     if (sr < 6000) {
//       // not black
//       Serial.print("OFF, go forward\n");
//       move_forward();
//     } else {
//       Serial.print("go forward\n");
//       move_forward();
//     }
//   } else if (lr_ratio <= 0.8) {
//     Serial.print("go right\n");
//     move_right();
//   } else {
//     Serial.print("go left\n");
//     move_left();
//   }
  
  // int sl = sensors[0];
  // int sr = sensors[5];

  // if (abs(sl - sr) < 300 && sr > 8000) {
  //   Serial.print("sees black, go forward\n");
  //   move_forward();
  // } 
  // else if (abs(sl - sr) < 300 && sr <= 8000) {
  //   Serial.print("sees nothing, but still go forward\n");
  //   move_forward();
  // } 
  // else if (sl > sr) {
  //   Serial.print("go left\n");
  //   move_left();
  // } else if (sr >= sl) {
  //   Serial.print("go right\n");
  //   move_right();
  // }


  // if (sl <= 1000 && sr <= 1000) {
  //   //black line in between
  //   move_forward();
  // }
  // if (sl > 1000 && sr <= 1000) {
  //   // curves left
  //   move_left();
  // }
  // if (sl <= 1000 && sr > 1000) {
  //   // curves right
  //   move_right();
  // }
  // if (sl > 1000 && sr > 1000) {
  //   move_forward();
  //   //or forward
  // }
// }

// void line_follow_stop(int sensors[6]) {
//   int sl = sensors[0];
//   int sr = sensors[6];
//   if (sl <= 5000 && sr <= 5000) {
//     //black line in between
//     move_forward();
//   }
//   if (sl > 5000 && sr <= 5000) {
//     // curves left
//     move_left();
//   }
//   if (sl <= 5000 && sr > 5000) {
//     // curves right
//     move_right();
//   }
//   if (sl > 5000 && sr > 5000) {
//     // issue 
//     stopDrivetrain();
//     delay(6000);
//     move_forward();
//     //or forward
//   }
// }




// vim: tabstop=2 shiftwidth=2 expandtab
