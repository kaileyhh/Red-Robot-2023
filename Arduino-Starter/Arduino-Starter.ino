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
    return max;
  }
  if (in < min) {
    return min;
  }
  return in;
}

void arcadeDrive(float thrust, float rotation) {
  // rotation = clamp(rotation, -0.5, 0.5);
  thrust = abs(thrust) > 0.005 ? thrust : 0;
  rotation = abs(rotation) > 0.005 ? rotation : 0;

  // thrust = pow(thrust, 3);
  // rotation = pow(rotation, 3);

  Serial.print(thrust);
  Serial.println();
  Serial.print(rotation);
  Serial.println();

  RR_setMotor1(thrust + rotation);
  RR_setMotor2(thrust - rotation);
}

void score_piece() {
  // run certain motor forwards
  // run certain motor backwards
}

int temp = 0;

void loop() {
  // Read the four joystick axes
  // These will be in the range [-1.0, 1.0]
  float rightX = RR_axisRX();
  float rightY = RR_axisRY();
  float leftX  = RR_axisLX();
  float leftY  = RR_axisLY();

  // Arcade-drive scheme
  // Left Y-axis = throttle
  // Right X-axis = steering
  // RR_setMotor1(leftY + rightX);
  // RR_setMotor2(leftY - rightX);

  arcadeDrive(leftY, rightX);

  // Get the button states
  bool btnA = RR_buttonA();
  bool btnB = RR_buttonB();
  bool btnX = RR_buttonX();
  bool btnY = RR_buttonY();
  bool btnRB = RR_buttonRB();
  bool btnLB = RR_buttonLB();

  // Control motor3 port (unused on base robot) using A/B buttons
  if (btnA) {
    Serial.write("HELLLOOOOO A \n");
    digitalWrite(LED_BUILTIN, HIGH);
    RR_setMotor3(1.0);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // score
  if (btnB) {
    score_piece();
  }
  // else {
  //   RR_setMotor3(0.0);
  // }

  // Control motor4 port (unused on base robot) using X/Y buttons
  if (btnX) {
    RR_setMotor4(1.0);
  }
  else if (btnY) {
    RR_setMotor4(-1.0);
  }
  else {
    RR_setMotor4(0.0);
  }

  // Control servo 1 using the dpad
  // 6 = left, 2 = right, 0 = up, 4 = down, 8 = center
  // (note that you will also see the value 0 if the controller
  //  is disconnected)
  if (RR_dpad() == 6) { // left

    // we can't move a servo less than 0 degrees
    if (temp > 0) temp -= 10;
  }
  else if (RR_dpad() == 2) { // right

    // we can't move a servo past 180 degrees
    // for continuous rotation, try using a DC motor
    if (temp < 180) temp += 10;
  }
  RR_setServo1(temp);

  // Control servo 2 using the shoulder buttons
  // This example moves the servo to fixed points
  // You can change the angles based on your mechanism
  // (this is great for a mechanism that only has 2 states,
  //  such as a grabber or hook)
  if (btnRB) {
    RR_setServo2(180);
  }
  else if (btnLB) {
    RR_setServo2(0);
  }

  // we also have RR_setServo3 and RR_setServo4 available


  // read the ultrasonic sensors

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

  // This is important - it sleeps for 0.02 seconds (= 50 times / second)
  // Running the code too fast will overwhelm the microcontroller and peripherals
  delay(20);
}

// void line_follow(int sensors[6]) {
//   //precondition: sensor is set such that the black line is in front

//   if (sl == LOW && sr == LOW) {
//     //black line in between
//     move_forward();
//   }
//   if (sl == HIGH && sr == LOW) {
//     // curves left
//     move_left();
//   }
//   if (sl == LOW && sr == HIGH) {
//     // curves right
//     move_right();
//   }
//   if (sl == HIGH && sr == HIGH) {
//     // issue 
//     stop_rr();
//     //or forward
//   }
// }

// void move_forward() {
//   // in development -- set it to go forward
//   RR_setMotor1(leftY + rightX);
//   RR_setMotor2(leftY - rightX);
// }

// void move_left() {
//   // in development -- set it to go forward
//   RR_setMotor1(leftY + rightX);
//   RR_setMotor2(leftY - rightX);
// }

// void move_right() {
//   // in development -- set it to go forward
//   RR_setMotor1(leftY + rightX);
//   RR_setMotor2(leftY - rightX);
// }

// void stop_rr() {
//   // in development -- set it to go forward
//   RR_setMotor1(0);
//   RR_setMotor2(0);
// }

// vim: tabstop=2 shiftwidth=2 expandtab
