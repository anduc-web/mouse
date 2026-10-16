#include <Arduino.h>
#include <math.h>

// ======================================================
// 1. CHAN STM32F411CEU6
// ======================================================

const uint8_t LEFT_IN1  = PA1;
const uint8_t LEFT_IN2  = PA2;
const uint8_t RIGHT_IN1 = PA3;
const uint8_t RIGHT_IN2 = PA5;

const uint8_t MOTOR_SLEEP = PA0;

const uint8_t ENC_LEFT_A  = PB5;
const uint8_t ENC_LEFT_B  = PB4;
const uint8_t ENC_RIGHT_A = PA8;
const uint8_t ENC_RIGHT_B = PA9;

const uint8_t LED_PIN = PC13;

const uint8_t LED_ON  = LOW;
const uint8_t LED_OFF = HIGH;


// ======================================================
// 2. HIEU CHUAN ENCODER VA QUANG DUONG
// ======================================================

const float WHEEL_DIAMETER_MM = 34.0f;
const float ENCODER_PPR = 7.0f;
const float GEAR_RATIO = 10.0f;

const float TARGET_DISTANCE_MM = 180.0f;

// Hieu chuan theo ket qua thu nghiem truoc:
// 118 xung trung binh tuong ung khoang 240 mm.
const float CALIBRATION_COUNTS = 118.0f;
const float CALIBRATION_DISTANCE_MM = 240.0f;

// DIEU CHINH: tang nguong phanh len 220 mm
const float BRAKE_TRIGGER_DISTANCE_MM = 225.0f;


// ======================================================
// 3. DIEU KHIEN MOTOR
// ======================================================

const int BASE_DUTY = 30;
const int MIN_DUTY  = 22;
const int MAX_DUTY  = 45;

// DIEU CHINH: tang bu PWM ben phai tu 3 len 4
const int MOTOR_TRIM_RIGHT = 6;

const int START_BOOST_ADD = 10;
const uint32_t START_BOOST_MS = 150;


// ======================================================
// 4. GIAM TOC TRUOC KHI PHANH
// ======================================================

const float SLOWDOWN_START_MM = 100.0f;
const float SLOWDOWN_END_MM = 145.0f;

const int MIN_APPROACH_DUTY = 22;


// ======================================================
// 5. CAN BANG ENCODER, KHONG DUNG PID
// ======================================================

const uint32_t BALANCE_PERIOD_MS = 120;

const int BALANCE_DEADBAND_PULSES = 1;
const int BALANCE_STEP = 1;
const int MAX_BALANCE_CORRECTION = 5;

int balanceCorrection = 0;

long lastBalanceLeft = 0;
long lastBalanceRight = 0;


// ======================================================
// 6. BAO VE
// ======================================================

const uint32_t MAX_RUN_TIME_MS = 8000;
const uint32_t ENCODER_STALL_TIMEOUT_MS = 1200;
const uint32_t BRAKE_HOLD_MS = 120;


// ======================================================
// 7. BIEN TOAN CUC
// ======================================================

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

bool running = false;
bool ledState = false;

uint32_t motionStartMs = 0;
uint32_t lastLedMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastBalanceMs = 0;

uint32_t lastLeftPulseMs = 0;
uint32_t lastRightPulseMs = 0;

long lastObservedLeft = 0;
long lastObservedRight = 0;


// ======================================================
// 8. NGAT ENCODER
// ======================================================

void encoderLeftISR() {
  encoderLeft++;
}

void encoderRightISR() {
  encoderRight++;
}


// ======================================================
// 9. DOC ENCODER AN TOAN
// ======================================================

void readEncoders(long &l, long &r) {
  noInterrupts();

  l = encoderLeft;
  r = encoderRight;

  interrupts();
}


// ======================================================
// 10. TINH QUANG DUONG TU SO XUNG
// ======================================================

float countsToDistanceMM(float counts) {
  return counts * CALIBRATION_DISTANCE_MM
         / CALIBRATION_COUNTS;
}


// ======================================================
// 11. DIEU KHIEN MOTOR
// ======================================================

void motorBrake() {
  // DRV8833: ca hai chan cua moi motor o HIGH

  analogWrite(LEFT_IN1, 255);
  analogWrite(LEFT_IN2, 255);

  analogWrite(RIGHT_IN1, 255);
  analogWrite(RIGHT_IN2, 255);
}

void motorCoast() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);

  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}

void setMotors(int dL, int dR) {
  dL = constrain(dL, 0, MAX_DUTY);
  dR = constrain(dR, 0, MAX_DUTY);

  int pwmL = map(dL, 0, 100, 0, 255);
  int pwmR = map(dR, 0, 100, 0, 255);

  // Banh trai tien
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, pwmL);

  // Banh phai tien
  analogWrite(RIGHT_IN1, pwmR);
  analogWrite(RIGHT_IN2, 0);
}


// ======================================================
// 12. VO HIEU HOA DRIVER
// ======================================================

void disableMotorDriver() {
  motorCoast();

  // PA0 phai duoc noi voi chan nSLEEP cua DRV8833
  digitalWrite(MOTOR_SLEEP, LOW);
}


// ======================================================
// 13. DUNG XE KHI DAT NGUONG PHANH
// ======================================================

void stopAtTarget(float distanceAtBrake) {
  running = false;

  // Phanh ngay khi dat nguong
  motorBrake();
  delay(BRAKE_HOLD_MS);

  // Tat PWM va vo hieu hoa driver theo code goc
  disableMotorDriver();

  long l, r;
  readEncoders(l, r);

  float avgCounts = ((float)l + (float)r) / 2.0f;
  float finalDistanceMM = countsToDistanceMM(avgCounts);

  Serial.println();
  Serial.println("========== TARGET BRAKE ==========");

  Serial.print("Left encoder: ");
  Serial.println(l);

  Serial.print("Right encoder: ");
  Serial.println(r);

  Serial.print("Average counts: ");
  Serial.println(avgCounts, 2);

  Serial.print("Distance at brake command: ");
  Serial.print(distanceAtBrake, 1);
  Serial.println(" mm");

  Serial.print("Distance after brake sequence: ");
  Serial.print(finalDistanceMM, 1);
  Serial.println(" mm");

  Serial.print("Desired distance: ");
  Serial.print(TARGET_DISTANCE_MM, 1);
  Serial.println(" mm");

  Serial.print("Brake threshold: ");
  Serial.print(BRAKE_TRIGGER_DISTANCE_MM, 1);
  Serial.println(" mm");

  Serial.println("Motor PWM disabled.");
  Serial.println("Driver sleep LOW.");
  Serial.println("==================================");

  digitalWrite(LED_PIN, LED_OFF);
}


// ======================================================
// 14. DUNG XE KHI CO LOI
// ======================================================

void stopWithError(int flashes, const char *message) {
  running = false;

  motorBrake();
  delay(BRAKE_HOLD_MS);

  disableMotorDriver();

  Serial.println();
  Serial.println(message);
  Serial.println("Motor PWM disabled.");
  Serial.println("Driver sleep LOW.");

  for (int i = 0; i < flashes; i++) {
    digitalWrite(LED_PIN, LED_ON);
    delay(180);

    digitalWrite(LED_PIN, LED_OFF);
    delay(180);
  }

  digitalWrite(LED_PIN, LED_OFF);
}


// ======================================================
// 15. CAN BANG HAI BANH BANG ENCODER, KHONG PID
// ======================================================

void updateStraightBalance(long l, long r, uint32_t now) {
  if (now - lastBalanceMs < BALANCE_PERIOD_MS) {
    return;
  }

  long deltaL = l - lastBalanceLeft;
  long deltaR = r - lastBalanceRight;

  lastBalanceLeft = l;
  lastBalanceRight = r;
  lastBalanceMs = now;

  if (deltaL > deltaR + BALANCE_DEADBAND_PULSES) {
    // Banh trai nhanh hon:
    // giam PWM trai, tang PWM phai.
    balanceCorrection += BALANCE_STEP;
  }
  else if (deltaR > deltaL + BALANCE_DEADBAND_PULSES) {
    // Banh phai nhanh hon:
    // giam PWM phai, tang PWM trai.
    balanceCorrection -= BALANCE_STEP;
  }
  else {
    // Hai banh gan bang nhau:
    // giam dan hieu chinh cu de tranh giu lech lau.
    if (balanceCorrection > 0) {
      balanceCorrection--;
    }
    else if (balanceCorrection < 0) {
      balanceCorrection++;
    }
  }

  balanceCorrection = constrain(
      balanceCorrection,
      -MAX_BALANCE_CORRECTION,
      MAX_BALANCE_CORRECTION
  );
}


// ======================================================
// 16. SETUP
// ======================================================

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(MOTOR_SLEEP, OUTPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);

  analogWriteResolution(8);

  digitalWrite(LED_PIN, LED_OFF);
  digitalWrite(MOTOR_SLEEP, LOW);

  motorCoast();

  // LED kiem tra
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LED_ON);
    delay(200);

    digitalWrite(LED_PIN, LED_OFF);
    delay(200);
  }

  // Bat driver
  digitalWrite(MOTOR_SLEEP, HIGH);
  delay(10);

  attachInterrupt(
      digitalPinToInterrupt(ENC_LEFT_A),
      encoderLeftISR,
      RISING
  );

  attachInterrupt(
      digitalPinToInterrupt(ENC_RIGHT_A),
      encoderRightISR,
      RISING
  );

  Serial.println();
  Serial.println("=== MICROMOUSE STRAIGHT + BRAKE TEST ===");
  Serial.println("PID: DISABLED");
  Serial.println("Control: encoder balance");

  Serial.print("Calibration counts: ");
  Serial.println(CALIBRATION_COUNTS, 1);

  Serial.print("Calibration distance: ");
  Serial.print(CALIBRATION_DISTANCE_MM, 1);
  Serial.println(" mm");

  Serial.print("Target distance: ");
  Serial.print(TARGET_DISTANCE_MM, 1);
  Serial.println(" mm");

  Serial.print("Brake threshold: ");
  Serial.print(BRAKE_TRIGGER_DISTANCE_MM, 1);
  Serial.println(" mm");

  Serial.print("Right motor trim: ");
  Serial.println(MOTOR_TRIM_RIGHT);

  Serial.println("Slowdown begins at 100 mm.");
  Serial.println("Low PWM begins at 145 mm.");
  Serial.println("Hard timeout: 8 seconds.");

  delay(2000);

  // Reset encoder truoc khi chay
  noInterrupts();

  encoderLeft = 0;
  encoderRight = 0;

  interrupts();

  balanceCorrection = 0;

  motionStartMs = millis();

  lastLedMs = motionStartMs;
  lastDebugMs = motionStartMs;
  lastBalanceMs = motionStartMs;

  lastLeftPulseMs = motionStartMs;
  lastRightPulseMs = motionStartMs;

  lastObservedLeft = 0;
  lastObservedRight = 0;

  lastBalanceLeft = 0;
  lastBalanceRight = 0;

  // Khoi dong motor
  int startL = constrain(
      BASE_DUTY + START_BOOST_ADD,
      MIN_DUTY,
      MAX_DUTY
  );

  int startR = constrain(
      BASE_DUTY + START_BOOST_ADD + MOTOR_TRIM_RIGHT,
      MIN_DUTY,
      MAX_DUTY
  );

  setMotors(startL, startR);

  running = true;

  Serial.println("MOTOR STARTED");
}


// ======================================================
// 17. LOOP
// ======================================================

void loop() {
  if (!running) {
    digitalWrite(LED_PIN, LED_OFF);
    return;
  }

  uint32_t now = millis();

  // ----------------------------------------------------
  // 1. TIMEOUT CUNG
  // ----------------------------------------------------

  if (now - motionStartMs >= MAX_RUN_TIME_MS) {
    stopWithError(5, "ERROR: HARD TIMEOUT - STOPPING");
    return;
  }

  // ----------------------------------------------------
  // 2. DOC ENCODER
  // ----------------------------------------------------

  long l, r;
  readEncoders(l, r);

  if (l != lastObservedLeft) {
    lastObservedLeft = l;
    lastLeftPulseMs = now;
  }

  if (r != lastObservedRight) {
    lastObservedRight = r;
    lastRightPulseMs = now;
  }

  // ----------------------------------------------------
  // 3. TINH QUANG DUONG
  // ----------------------------------------------------

  float avgCounts = ((float)l + (float)r) / 2.0f;
  float distanceMM = countsToDistanceMM(avgCounts);

  // ----------------------------------------------------
  // 4. IN THONG TIN MOI 100 MS
  // ----------------------------------------------------

  if (now - lastDebugMs >= 100) {
    lastDebugMs = now;

    Serial.print("L=");
    Serial.print(l);

    Serial.print(" R=");
    Serial.print(r);

    Serial.print(" Avg=");
    Serial.print(avgCounts, 2);

    Serial.print(" S=");
    Serial.print(distanceMM, 1);

    Serial.print(" mm Correction=");
    Serial.println(balanceCorrection);
  }

  // ----------------------------------------------------
  // 5. PHANH KHI DAT NGUONG 220 MM
  // ----------------------------------------------------

  if (distanceMM >= BRAKE_TRIGGER_DISTANCE_MM) {
    stopAtTarget(distanceMM);
    return;
  }

  // ----------------------------------------------------
  // 6. BAO LOI ENCODER
  // ----------------------------------------------------

  if (now - lastLeftPulseMs >= ENCODER_STALL_TIMEOUT_MS) {
    stopWithError(3, "ERROR: LEFT ENCODER STALL - STOPPING");
    return;
  }

  if (now - lastRightPulseMs >= ENCODER_STALL_TIMEOUT_MS) {
    stopWithError(4, "ERROR: RIGHT ENCODER STALL - STOPPING");
    return;
  }

  // ----------------------------------------------------
  // 7. CAN BANG HAI BANH KHONG DUNG PID
  // ----------------------------------------------------

  updateStraightBalance(l, r, now);

  // ----------------------------------------------------
  // 8. PWM CO BAN VA TANG LUC KHOI DONG
  // ----------------------------------------------------

  float centerDuty = (float)BASE_DUTY;

  uint32_t elapsedMotion = now - motionStartMs;

  if (elapsedMotion < START_BOOST_MS) {
    float boostFactor =
        1.0f -
        (float)elapsedMotion / (float)START_BOOST_MS;

    centerDuty += (float)START_BOOST_ADD * boostFactor;
  }

  // ----------------------------------------------------
  // 9. GIAM TOC TU 100 MM DEN 145 MM
  // ----------------------------------------------------

  if (distanceMM > SLOWDOWN_START_MM &&
      distanceMM < SLOWDOWN_END_MM) {

    float ratio =
        (SLOWDOWN_END_MM - distanceMM) /
        (SLOWDOWN_END_MM - SLOWDOWN_START_MM);

    ratio = constrain(ratio, 0.0f, 1.0f);

    float decelDuty =
        (float)MIN_APPROACH_DUTY +
        ((float)BASE_DUTY - (float)MIN_APPROACH_DUTY)
        * ratio;

    if (decelDuty < centerDuty) {
      centerDuty = decelDuty;
    }
  }

  // Tu 145 mm den nguong phanh:
  // duy tri PWM thap nhat.

  if (distanceMM >= SLOWDOWN_END_MM &&
      distanceMM < BRAKE_TRIGGER_DISTANCE_MM) {
    centerDuty = MIN_APPROACH_DUTY;
  }

  // ----------------------------------------------------
  // 10. DIEU CHINH PWM DE XE DI THANG
  // ----------------------------------------------------

  float leftDuty =
      centerDuty - (float)balanceCorrection;

  float rightDuty =
      centerDuty +
      (float)MOTOR_TRIM_RIGHT +
      (float)balanceCorrection;

  int dL = (int)roundf(leftDuty);
  int dR = (int)roundf(rightDuty);

  dL = constrain(dL, MIN_DUTY, MAX_DUTY);
  dR = constrain(dR, MIN_DUTY, MAX_DUTY);

  setMotors(dL, dR);

  // ----------------------------------------------------
  // 11. LED NHAP NHAY
  // ----------------------------------------------------

  if (now - lastLedMs >= 300) {
    lastLedMs = now;

    ledState = !ledState;

    digitalWrite(
        LED_PIN,
        ledState ? LED_ON : LED_OFF
    );
  }
}