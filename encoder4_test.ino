#include <Wire.h>
#include <AS5600.h>
#include <TCA9548A.h>

// ─────────── Multiplexer ───────────
TCA9548A tca(0x70);

// ─────────── Encoders ───────────
AS5600 encoder[4];

// Raw + Total Count Storage
uint16_t prevRaw[4] = {0,0,0,0};
long encoderCount[4] = {0,0,0,0};

// ─────────── PWM Pins (Safety LOW) ───────────
const int PWM_PINS[8] = {2,3,4,5,6,7,10,11};

// ─────────── ENABLE Pins (Safety LOW) ───────────
const int EN_PINS[8]  = {22,23,24,25,26,27,28,29};

void setup() {

  Serial.begin(9600);
  Wire.begin();

  // ───── Keep ALL PWM LOW
  for(int i=0;i<8;i++){
    pinMode(PWM_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
  }

  // ───── Keep ALL ENABLE LOW (Drivers Disabled)
  for(int i=0;i<8;i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW);
  }
  delay(500);

  // ───── Initialize Multiplexer
  tca.begin();

  // ───── Initialize all 4 encoders
  for(int ch=0; ch<4; ch++){
    tca.openChannel(ch);
    encoder[ch].begin();
    delay(10);

    prevRaw[ch] = encoder[ch].readAngle();
    encoderCount[ch] = 0;

    tca.closeChannel(ch);     
  }

  Serial.println("==== 4 Encoder Test Ready ====");
  Serial.println("Type e and press Enter");
}   
void loop() { 

  updateEncoders();

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "e") {

      Serial.print("RF:");
      Serial.print(encoderCount[0]);   

      Serial.print("  LF:");    
      Serial.print(-encoderCount[1]);

      Serial.print("  LR:");
      Serial.print(-encoderCount[2]);
      Serial.print("  RR:");  
      Serial.println(encoderCount[3]);
    }
  }
}


// ─────────── Update Encoder Counts ───────────

void updateEncoders(){

  for(int ch=0; ch<4; ch++){ 

    tca.openChannel(ch);

    uint16_t raw = encoder[ch].readAngle();

    int delta = raw - prevRaw[ch];
    // Handle 0–4095 rollover
    if(delta > 2048) delta -= 4096; 
    else if(delta < -2048) delta += 4096;

    encoderCount[ch] += (delta);
    prevRaw[ch] = raw;

    tca.closeChannel(ch);
  } 
}   
