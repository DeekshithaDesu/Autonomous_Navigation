#include <Wire.h>
#define PCA9548A_ADDR 0x70 
#define AS5600_ADDR 0x36   
#define RAW_ANGLE_REG 0x0C 

// =====================================================================
// SmartMotor Class 
// =====================================================================
class SmartMotor {
  private:
    int R_EN, L_EN, R_PWM, L_PWM;
    uint8_t muxChannel;
    float zeroAngle;
    float gearRatio;
    float wheelCircumference_m;
    bool inverted; 

    unsigned long lastTime;
    float lastAngle;
    float filteredWheelRPM; 
    const float filterAlpha = 0.2; 

    float kp, ki, kd;
    float integralError, lastError;
    unsigned long lastIntegralResetTime; 

    float cumulativeDistance_m;

    void selectMuxChannel() {
      Wire.beginTransmission(PCA9548A_ADDR);
      Wire.write(1 << muxChannel); 
      Wire.endTransmission();
    }

    float readAngle() {
      selectMuxChannel(); 
      Wire.beginTransmission(AS5600_ADDR);
      Wire.write(RAW_ANGLE_REG);
      Wire.endTransmission(false);
      Wire.requestFrom(AS5600_ADDR, 2);
      if (Wire.available() == 2) {
        uint16_t highByte = Wire.read();
        uint16_t lowByte = Wire.read();
        return (float)((highByte << 8) | lowByte);
      }
      return -1.0; 
    }

  public:
    int currentPWM; 
    float getRawAngle() {
      return readAngle() - zeroAngle;
    }

    SmartMotor(int r_en, int l_en, int r_pwm, int l_pwm, uint8_t mux_ch, 
               float g_ratio, float w_circ_m, float p, float i, float d, bool inv = false) {
      R_EN = r_en; L_EN = l_en; R_PWM = r_pwm; L_PWM = l_pwm;
      muxChannel = mux_ch; gearRatio = g_ratio; wheelCircumference_m = w_circ_m;
      kp = p; ki = i; kd = d; inverted = inv;
      
      lastTime = 0; lastAngle = 0; filteredWheelRPM = 0;
      integralError = 0; lastError = 0; lastIntegralResetTime = 0;
      currentPWM = 0; cumulativeDistance_m = 0;
    }

    void begin() {
      pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
      pinMode(R_PWM, OUTPUT); pinMode(L_PWM, OUTPUT);
      stop();
      lastAngle = readAngle();                                            
      zeroAngle = lastAngle;
      cumulativeDistance_m = 0; 
      filteredWheelRPM = 0;
      integralError = 0;
      lastError = 0;   
      lastTime = millis();
      lastIntegralResetTime = millis();
    }

    void resetOdometry() {
      cumulativeDistance_m = 0;
    }

    float getCumulativeDistance() {
      return cumulativeDistance_m;
    }

    void run(float targetWheelRPM) {
      unsigned long currentTime = millis();
      float deltaTime_sec = (currentTime - lastTime) / 1000.0;

      if (deltaTime_sec < 0.01) return; 

      float currentAngle = readAngle();
      if (currentAngle != -1.0) {
        float deltaAngle = currentAngle - lastAngle;
        
        if (deltaAngle > 2048.0) deltaAngle -= 4096.0;
        else if (deltaAngle < -2048.0) deltaAngle += 4096.0;

        if (inverted) deltaAngle = -deltaAngle;

        float motorRevs = deltaAngle / 4096.0;
        float wheelRevs = motorRevs / gearRatio;
        
        cumulativeDistance_m += (wheelRevs * wheelCircumference_m);

        float rawWheelRPM = wheelRevs / (deltaTime_sec / 60.0);
        filteredWheelRPM = (filterAlpha * rawWheelRPM) + ((1.0 - filterAlpha) * filteredWheelRPM);
        lastAngle = currentAngle;
      }

      if (targetWheelRPM == 0) {
        stop();
        lastTime = currentTime; 
        lastIntegralResetTime = currentTime;
        return;
      }

      float error = targetWheelRPM - filteredWheelRPM;
      if (currentTime - lastIntegralResetTime >= 5000) {
        integralError = 0;
        lastIntegralResetTime = currentTime;
      }

      integralError += (error * deltaTime_sec);
      integralError = constrain(integralError, -1000.0, 1000.0); 
      float derivativeError = (error - lastError) / deltaTime_sec;

      float controlSignal = (kp * error) + (ki * integralError) + (kd * derivativeError);
      lastError = error;
      lastTime = currentTime;

      if (inverted) controlSignal = -controlSignal;
      currentPWM = constrain((int)controlSignal, -255, 255);
      digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);

      if (currentPWM > 0) {
        analogWrite(R_PWM, abs(currentPWM));
        analogWrite(L_PWM, 0);
      } else {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, abs(currentPWM));
      }
    }

    void stop() {
      analogWrite(R_PWM, 0); analogWrite(L_PWM, 0);
      digitalWrite(R_EN, LOW); digitalWrite(L_EN, LOW);
      integralError = 0; lastError = 0; currentPWM = 0;
      lastIntegralResetTime = millis();
    }
    float getRPM() { return filteredWheelRPM; } 
};


const float GEAR_RATIO = 3.6;
const float WHEEL_CIRC_METERS = 0.3400;


// =====================================================================
// Rover Class 
// =====================================================================
class Rover {
  private:
    SmartMotor *rf, *lf, *lr, *rr;   
    
    float trackWidth_m; 
    float wheelBase_m;  
    float diagonal_m;   
    float turnRadius_m; 
    unsigned long lastPrintTime = 0;
    enum State { IDLE, MOVING_STRAIGHT, TURNING } currentState;
    float targetDistance_m;
    float maneuverRPM; 
    float rpmRF, rpmLF, rpmLR, rpmRR;

    // Relative maneuver tracking states
    float startLeftDistance = 0.0;
    float startRightDistance = 0.0;

  public:
    Rover(SmartMotor* _rf, SmartMotor* _lf, SmartMotor* _lr, SmartMotor* _rr, 
          float _trackWidth_m, float _wheelBase_m) {
      rf = _rf; lf = _lf; lr = _lr; rr = _rr;
      trackWidth_m = _trackWidth_m;
      wheelBase_m = _wheelBase_m;
      
      diagonal_m = sqrt(pow(trackWidth_m, 2) + pow(wheelBase_m, 2));
      turnRadius_m=0.24;
      // turnRadius_m = diagonal_m / 2.0;   

      currentState = IDLE;
      maneuverRPM = 40.0;   
      rpmRF = 0; rpmLF = 0; rpmLR = 0; rpmRR = 0;
    }

    // --- Continuous Cumulative Odometry Trackers (SIGNED FOR ROS) ---
    float getLeftDistance() {
        return (lf->getCumulativeDistance() + lr->getCumulativeDistance()) * 0.5;
    }

    float getRightDistance() {
        return (rf->getCumulativeDistance() + rr->getCumulativeDistance()) * 0.5;
    }

    float getLeftRPM() {
        return (lf->getRPM() + lr->getRPM()) * 0.5;
    }

    float getRightRPM() {
      return (rf->getRPM() + rr->getRPM()) * 0.5;
    }
    
    void setVelocity(float linearVel, float angularVel)
    {
    
      float leftVel =
          linearVel -
          (angularVel * trackWidth_m / 2.0);

      float rightVel =
          linearVel +
          (angularVel * trackWidth_m / 2.0);

      float leftRPM =
          (leftVel / WHEEL_CIRC_METERS) * 60.0;

      float rightRPM =
          (rightVel / WHEEL_CIRC_METERS) * 60.0;

      rpmLF = constrain(leftRPM, -40, 40);
      rpmLR = constrain(leftRPM, -40, 40);

      rpmRF = constrain(rightRPM, -40, 40);
      rpmRR = constrain(rightRPM, -40, 40);

      currentState = IDLE;
    }

    void setManeuverSpeed(float rpm) { maneuverRPM = abs(rpm); }

    void stop(String source = "") {
      currentState = IDLE;
      rpmRF = 0; rpmLF = 0; rpmLR = 0; rpmRR = 0;
      String msg = "ROVER: Stopped.";
      Serial.println(msg);
      if(source == "BT") Serial1.println(msg);
    }

    void moveStraight(float distance_m, bool forward, String source = "") {
      targetDistance_m = abs(distance_m);
      
      startLeftDistance = getLeftDistance();
      startRightDistance = getRightDistance();
      
      float speed = forward ? maneuverRPM : -maneuverRPM;
      rpmRF = speed; rpmLF = speed; rpmLR = speed; rpmRR = speed;
      
      currentState = MOVING_STRAIGHT;
      String msg = "ROVER: Moving " + String(forward ? "Forward " : "Backward ") + String(targetDistance_m) + " meters.";
      Serial.println(msg);
      if(source == "BT") Serial1.println(msg);
    }

    void turn(float degrees, bool rightTurn, String source = "") {
      float radians = abs(degrees) * (PI / 180.0);
      
      // SKID FACTOR APPLIED HERE for accurate manual Bluetooth turns!
      float skidFactor = 1.3; 
      targetDistance_m = radians * (turnRadius_m * skidFactor); 
      
      startLeftDistance = getLeftDistance();
      startRightDistance = getRightDistance();

      if (rightTurn) {
        rpmLF = maneuverRPM; rpmLR = maneuverRPM;
        rpmRF = -maneuverRPM; rpmRR = -maneuverRPM;
      } else {
        rpmLF = -maneuverRPM; rpmLR = -maneuverRPM;
        rpmRF = maneuverRPM; rpmRR = maneuverRPM;
      }

      currentState = TURNING;
      
      String msg = "ROVER: Turning " + String(rightTurn ? "Right " : "Left ") + String(abs(degrees)) + " degrees.";
      Serial.println(msg);
      if(source == "BT") Serial1.println(msg);
    }

    void update() {

      // First update all motors and encoder values
      rf->run(rpmRF);
      lf->run(rpmLF);
      lr->run(rpmLR);
      rr->run(rpmRR);

      // Then check maneuver completion
      if (currentState != IDLE) {

        float leftDistance =
            getLeftDistance() - startLeftDistance;

        float rightDistance =
            getRightDistance() - startRightDistance;

        float avgDistTraveled =
            (abs(leftDistance) + abs(rightDistance)) / 2.0;
        Serial1.print("Target=");
        Serial1.print(targetDistance_m);  
      
        Serial1.print(" Left=");
        Serial1.print(leftDistance);

        Serial1.print(" Right=");
        Serial1.print(rightDistance);

        Serial1.print(" Avg=");
        Serial1.println(avgDistTraveled);

        if (avgDistTraveled >= targetDistance_m) {

            Serial.println(
                "ROVER: Maneuver Complete."
            );

            Serial1.println(
                "ROVER: Maneuver Complete."
            );

            stop();
        }
      }

      if (millis() - lastPrintTime >= 100) {

        lastPrintTime = millis();

        float leftDist = getLeftDistance();
        float rightDist = getRightDistance();

        float leftRPM = getLeftRPM();
        float rightRPM = getRightRPM();

        Serial.println(
            String("ODOM,") +
            String(millis()) + "," +
            String(leftDist, 4) + "," +
            String(rightDist, 4) + "," +
            String(leftRPM, 2) + "," +
            String(rightRPM, 2)
        );

        Serial1.println(
            String("ODOM,") +
            String(millis()) + "," +
            String(leftDist, 4) + "," +
            String(rightDist, 4) + "," +
            String(leftRPM, 2) + "," +
            String(rightRPM, 2)
        );
      }
      
    }
   
};

// =====================================================================
// Implementation & Dual Serial Setup
// =====================================================================

const float TRACK_WIDTH_M = 0.55; 
const float WHEEL_BASE_M = 0.22; 

float Kp = 2.0; float Ki = 0.5; float Kd = 0.05; 

SmartMotor motorRF(23, 22, 2, 3, 2, GEAR_RATIO, WHEEL_CIRC_METERS, Kp, Ki, Kd, false); 
SmartMotor motorLF(24, 25, 4, 5, 3, GEAR_RATIO, WHEEL_CIRC_METERS, Kp, Ki, Kd, true);
SmartMotor motorLR(26, 27, 6, 7, 4, GEAR_RATIO, WHEEL_CIRC_METERS, Kp, Ki, Kd, true);
SmartMotor motorRR(28, 29, 10, 11, 5, GEAR_RATIO, WHEEL_CIRC_METERS, Kp, Ki, Kd, false); 

Rover myRover(&motorRF, &motorLF, &motorLR, &motorRR, TRACK_WIDTH_M, WHEEL_BASE_M);

String usbBuffer = "";
String btBuffer = "";

void setup() {
  Serial.begin(115200); 
  Serial1.begin(9600); 
  
  Serial.println("Dual-Serial Rover Ready. (USB: 115200 | BT: 9600)");
  Serial1.println("Bluetooth Connected. Send commands: 'f 1.5', 'r 90', 's'");
  
  Wire.begin(); 
  Wire.setClock(400000); 

  motorRF.begin(); motorLF.begin(); motorLR.begin(); motorRR.begin();
  myRover.setManeuverSpeed(40.0); 
}

void loop(){
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(usbBuffer, "USB");
      usbBuffer = ""; 
    } else if (c != '\r') {
      usbBuffer += c;
    }
  }

  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      processCommand(btBuffer, "BT");
      btBuffer = ""; 
    } else if (c != '\r') {
      btBuffer += c;
    }
  }

  myRover.update();
}

void processCommand(String input, String source) {
  Serial1.println("Recieved....successfully");
  Serial1.println(input);
  input.trim();
  input.toLowerCase();

  if (input.length() > 0) {
    char cmd = input.charAt(0);
    float val = 0.0;
    
    if (input.length() > 1) {
      val = input.substring(2).toFloat();
    }

    if (cmd == 's') myRover.stop(source);
    else if (cmd == 'f') myRover.moveStraight(val, true, source);
    else if (cmd == 'b') myRover.moveStraight(val, false, source);
    else if (cmd == 'r') myRover.turn(val, true, source);
    else if (cmd == 'l') myRover.turn(val, false, source);
    else if(cmd == 'v')
    {
      int spacePos = input.indexOf(' ',2);

      float linear =
          input.substring(2,spacePos).toFloat();

      float angular =
          input.substring(spacePos+1).toFloat();

      myRover.setVelocity(linear, angular);

      String velMsg =
          "VEL CMD: linear=" +
          String(linear) +
          " angular=" +
          String(angular);

      // Serial.println(velMsg);
      Serial1.println(velMsg);
    }
    else {
      if(source == "USB") Serial.println("Unknown command!");
      if(source == "BT") Serial1.println("Unknown command!");   
    }

  }
 
}
