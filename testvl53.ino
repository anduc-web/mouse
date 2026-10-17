#include <Arduino.h>
#include <Wire.h>

// DUNG PB9, PB8 KHONG CO DAU GACH DUOI
//             SDA  SCL
TwoWire VL53_I2C(PB9, PB8);

#define XSHUT_PIN PB12
#define VL53_ADDR 0x29

void scanI2C() {
  uint8_t count = 0;

  Serial.println("\n===== SCAN I2C PB8/PB9 =====");

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    VL53_I2C.beginTransmission(addr);
    uint8_t error = VL53_I2C.endTransmission();

    if (error == 0) {
      Serial.print("ACK at 0x");
      if (addr < 0x10) Serial.print('0');
      Serial.println(addr, HEX);
      count++;
    }
  }

  Serial.print("Devices found: ");
  Serial.println(count);
}

void checkVL53L0X() {
  Serial.println("\n===== CHECK VL53L0X =====");

  VL53_I2C.beginTransmission(VL53_ADDR);
  uint8_t error = VL53_I2C.endTransmission();

  if (error != 0) {
    Serial.print("I2C error = ");
    Serial.println(error);
    return;
  }

  Serial.println("ACK OK at 0x29");

  // Doc thanh ghi MODEL_ID 0xC0
  VL53_I2C.beginTransmission(VL53_ADDR);
  VL53_I2C.write(0xC0);

  error = VL53_I2C.endTransmission(false);

  if (error != 0) {
    Serial.print("Register address error = ");
    Serial.println(error);
    return;
  }

  uint8_t received = VL53_I2C.requestFrom(
    (uint8_t)VL53_ADDR,
    (uint8_t)1
  );

  if (received != 1 || !VL53_I2C.available()) {
    Serial.println("Cannot read MODEL_ID");
    return;
  }

  uint8_t id = VL53_I2C.read();

  Serial.print("MODEL_ID = 0x");
  if (id < 0x10) Serial.print('0');
  Serial.println(id, HEX);

  if (id == 0xEE) {
    Serial.println("SUCCESS: VL53L0X DETECTED");
  } else {
    Serial.println("Unexpected MODEL_ID");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nSTM32F411CEU6");
  Serial.println("Board: Generic F411CEUx");
  Serial.println("SDA = PB9");
  Serial.println("SCL = PB8");
  Serial.println("XSHUT = PB12");

  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(20);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(200);

  VL53_I2C.begin();
  VL53_I2C.setClock(100000);

  scanI2C();
  checkVL53L0X();
}

void loop() {
  delay(2000);
  checkVL53L0X();
}