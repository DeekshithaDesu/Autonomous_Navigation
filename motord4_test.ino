// ==================================================
// IBT-2 (BTS7960) 4 Motor Control
// Command: m 100 100 100 100
// Motor order: LF, RF, LR, RR
// ==================================================

// ---------- Enable Pins (UPDATED) ----------
const int R_EN[4] = {23, 25, 27, 29};
const int L_EN[4] = {22, 24, 26, 28};

// ---------- PWM Pins ----------
const int R_PWM[4] = {2, 4, 7, 11};
const int L_PWM[4] = {3, 5, 6, 10};   
void setup() {

  Serial.begin(9600); 

  // Initialize all pins
  for (int i = 0; i < 4; i++) {

    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    pinMode(R_PWM[i], OUTPUT);
    pinMode(L_PWM[i], OUTPUT);

    // Force LOW at startup
    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);

    analogWrite(R_PWM[i], 0);
    analogWrite(L_PWM[i], 0);
  }

  Serial.println("==== 4 Motor IBT-2 Ready ====");
  Serial.println("Format: m 100 100 100 100");
}

void loop() {

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("m")) {

      int s1, s2, s3, s4;

      int values = sscanf(cmd.c_str(), "m %d %d %d %d", &s1, &s2, &s3, &s4);

      if (values == 4) {

        int speed[4] = {s1, s2, s3, s4};

        for (int i = 0; i < 4; i++) {

          speed[i] = constrain(speed[i], -255, 255);

          if (speed[i] > 0) {

            digitalWrite(R_EN[i], HIGH);
            digitalWrite(L_EN[i], HIGH);

            // LEFT SIDE MOTORS (0,2)
            if (i == 0 || i == 2) {
              analogWrite(R_PWM[i], speed[i]);
              analogWrite(L_PWM[i], 0);
            }
            // RIGHT SIDE MOTORS (1,3)
            else {
              analogWrite(R_PWM[i], 0);
              analogWrite(L_PWM[i], speed[i]);
            }
          }

          else if (speed[i] < 0) {

            digitalWrite(R_EN[i], HIGH);
            digitalWrite(L_EN[i], HIGH);

            int revSpeed = -speed[i];

            // LEFT SIDE MOTORS (0,2)
            if (i == 0 || i == 2) {
              analogWrite(R_PWM[i], 0);
              analogWrite(L_PWM[i], revSpeed);
            }
            // RIGHT SIDE MOTORS (1,3)
            else {
              analogWrite(R_PWM[i], revSpeed);
              analogWrite(L_PWM[i], 0);
            }
          }

          else {

            analogWrite(R_PWM[i], 0);
            analogWrite(L_PWM[i], 0);

            digitalWrite(R_EN[i], LOW);
            digitalWrite(L_EN[i], LOW);
          }
        }

        Serial.println("Motors Updated");
      }                        
      else { 
        Serial.println("Invalid Format!");
      }
    }
  }
}
