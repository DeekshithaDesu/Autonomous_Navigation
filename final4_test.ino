#include <Wire.h>
#include <AS5600.h>
#include <TCA9548A.h> 
// ---------------- MOTOR SECTION ----------------

// Enable Pins
const int R_EN[4] = {23, 25, 27, 29};
const int L_EN[4] = {22, 24, 26, 28};

// PWM Pins (SWAPPED for Motor 1 & 2 in hardware)
const int R_PWM[4] = {3, 5, 6, 10};
const int L_PWM[4] = {2, 4, 7, 11};


// ---------------- ENCODER SECTION ----------------

// Multiplexer
TCA9548A tca(0x70);

// Encoders
AS5600 encoder[4];

uint16_t prevRaw[4] = {0,0,0,0};
long encoderCount[4] = {0,0,0,0};


// ==================================================

void setup() {

  Serial.begin(9600);
  Wire.begin();

  // ---- Initialize Motor Pins ----
  for (int i = 0; i < 4; i++) {

    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    pinMode(R_PWM[i], OUTPUT);
    pinMode(L_PWM[i], OUTPUT);

    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);

    analogWrite(R_PWM[i], 0);
    analogWrite(L_PWM[i], 0);
  }

  // ---- Initialize Multiplexer ----
  tca.begin();
  delay(200);

  // ---- Initialize Encoders ----
  for(int ch = 0; ch < 4; ch++) {

    tca.openChannel(ch);
    encoder[ch].begin();
    delay(10);

    prevRaw[ch] = encoder[ch].readAngle();
    encoderCount[ch] = 0;

    tca.closeChannel(ch);
  }

  Serial.println("====System Ready (e->encoder, m->system mvs) ====");
 
}


// ==================================================

void loop() {

  updateEncoders();   // Always update encoder in background

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // ---------- MOTOR COMMAND ----------
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

            if (i == 0 || i == 2) {
              analogWrite(R_PWM[i], speed[i]);
              analogWrite(L_PWM[i], 0);
            } else {
              analogWrite(R_PWM[i], 0);
              analogWrite(L_PWM[i], speed[i]);
            }
          }

          else if (speed[i] < 0) {

            digitalWrite(R_EN[i], HIGH);
            digitalWrite(L_EN[i], HIGH);

            int revSpeed = -speed[i];

            if (i == 0 || i == 2) {
              analogWrite(R_PWM[i], 0);
              analogWrite(L_PWM[i], revSpeed);
            } else {
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
        Serial.println("Invalid Motor Format!");
      }
    }


    // ---------- ENCODER PRINT COMMAND ----------
    else if (cmd == "e") {

      Serial.print("LF:");
      Serial.print(encoderCount[0]);

      Serial.print("  RF:");
      Serial.print(encoderCount[1]);

      Serial.print("  LR:");
      Serial.print(encoderCount[2]);

      Serial.print("  RR:");
      Serial.println(encoderCount[3]);
    }
  }
}


// ==================================================
// UPDATE ENCODERS FUNCTION
// ==================================================

void updateEncoders() {

  for(int ch = 0; ch < 4; ch++) {

    tca.openChannel(ch);

    uint16_t raw = encoder[ch].readAngle();
    int delta = raw - prevRaw[ch];

    // Handle rollover (0–4095)
    if(delta > 2048) delta -= 4096;
    else if(delta < -2048) delta += 4096;

    encoderCount[ch] += delta;
    prevRaw[ch] = raw;    
    tca.closeChannel(ch);     


















































































































    