#include "constants.h"

// Replace 12345 with the correct team number and then uncomment the line below.
#define TEAM_NUMBER 8

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 20 < TEAM_NUMBER
#error "Team number must be within 1 and 20"
#endif


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

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

void setServo(int angle) {
  Serial.print("setting servo: ");
  angle = clamp_int(angle, 0, 180);
  Serial.print(angle);
  Serial.print("\n");
  RR_setServo3(angle);
}

void setServoIntake() {
  setServo(INTAKE_SERVO_ANGLE);
}

void setServoScore() {
  setServo(SCORE_SERVO_ANGLE);
}

void setServoDefault() {
  setServo(DEFAULT_SERVO_ANGLE);
}

float getUltrasonic() {
  return RR_getUltrasonic();
}

bool isUltrasonicActivated() {
  // make 100.0 a real number;
  return getUltrasonic() > 100.0;
}

void stopDrivetrain() {
  RR_setMotor1(0.0);
  RR_setMotor2(0.0);
}

void arcadeDrive(float thrust, float rotation) {
  // rotation = clamp(rotation, -0.5, 0.5);
  thrust = abs(thrust) > 0.005 ? thrust : 0;
  rotation = abs(rotation) > 0.005 ? rotation : 0;

  RR_setMotor1(thrust + rotation);
  RR_setMotor2(thrust - rotation);
}

void debugPrints(bool btnA, bool btnB, bool btnX, bool btnY) {
  Serial.print("Ultrasonic=");
  Serial.print(RR_getUltrasonic());
  Serial.print(" ;; ");
  int sensors[6];

  Serial.print("Line sensors=");
  RR_getLineSensors(sensors);
  for (int i = 0; i < 6; ++i) {
    Serial.print(sensors[i]);
    Serial.print(" ");
  }


  //follows line
  // line_follow(sensors);

  Serial.print(btnA ? 1 : 0);
  Serial.print(btnB ? 1 : 0);
  Serial.print(btnX ? 1 : 0);
  Serial.print(btnY ? 1 : 0);
  Serial.println();
}

/* --- AUTO COMMANDS --- */

void scorePieceAuto() {
  setServoScore();
  delay(SCORE_TIME);
  setServoDefault();
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


  if (btnA) {
    Serial.write("HELLLOOOOO A \n");
    digitalWrite(LED_BUILTIN, HIGH);
    RR_setMotor3(1.0);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // left bumper - intake
  if (btnLB) {
    setServo(0);
  }
  // right bumper - score
  else if (btnRB) {
    setServo(180);
  } else {
    setServo(90);
  }

  if (btnX) {
    move_left();
  } else if (btnY) {
    move_right();
  } else if (btnB) {
    move_forward();
  }

  // read the ultrasonic sensors
  // debugPrints(btnA, btnB, btnX, btnY);

  // This is important - it sleeps for 0.02 seconds (= 50 times / second)
  // Running the code too fast will overwhelm the microcontroller and peripherals
  delay(20);
}

void line_follow(int sensors[6]) {
  //precondition: sensor is set such that the black line is in front
  int sl = sensors[0];
  int sr = sensors[6];
  if (sl <= 5000 && sr <= 5000) {
    //black line in between
    move_forward();
  }
  if (sl > 5000 && sr <= 5000) {
    // curves left
    move_left();
  }
  if (sl <= 5000 && sr > 5000) {
    // curves right
    move_right();
  }
  if (sl > 5000 && sr > 5000) {
    move_forward();
    //or forward
  }
}

void line_follow_stop(int sensors[6]) {
  int sl = sensors[0];
  int sr = sensors[6];
  if (sl <= 5000 && sr <= 5000) {
    //black line in between
    move_forward();
  }
  if (sl > 5000 && sr <= 5000) {
    // curves left
    move_left();
  }
  if (sl <= 5000 && sr > 5000) {
    // curves right
    move_right();
  }
  if (sl > 5000 && sr > 5000) {
    // issue 
    stopDrivetrain();
    delay(6000);
    move_forward();
    //or forward
  }
}




// vim: tabstop=2 shiftwidth=2 expandtab
