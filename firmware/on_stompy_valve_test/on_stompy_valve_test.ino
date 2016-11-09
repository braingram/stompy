#define M1_EN_PIN 8
#define M2_EN_PIN 2

#define HIP_PWM_0 9  // extend
#define HIP_PWM_1 10  // retract
#define THIGH_PWM_0 3  // extend
#define THIGH_PWM_1 4  // retract
#define KNEE_PWM_0 5  // extend
#define KNEE_PWM_1 6  // retract

#define ANALOG_WRITE_RES 15
#define ANALOG_WRITE_FREQ 1098.632

#define PWM_VALUE 6552

void enable_motors() {
  digitalWrite(M1_EN_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);
}

void disable_motors() {
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
}

void close_valves() {
  analogWrite(HIP_PWM_0, 0);
  analogWrite(HIP_PWM_1, 0);
  analogWrite(THIGH_PWM_0, 0);
  analogWrite(THIGH_PWM_1, 0);
  analogWrite(KNEE_PWM_0, 0);
  analogWrite(KNEE_PWM_1, 0);
}

void setup() {
  // disable motor outputs
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  disable_motors();
  analogWriteResolution(ANALOG_WRITE_RES);
  analogWriteFrequency(HIP_PWM_0, ANALOG_WRITE_FREQ);
  analogWriteFrequency(HIP_PWM_1, ANALOG_WRITE_FREQ);
  analogWriteFrequency(THIGH_PWM_0, ANALOG_WRITE_FREQ);
  analogWriteFrequency(THIGH_PWM_1, ANALOG_WRITE_FREQ);
  analogWriteFrequency(KNEE_PWM_0, ANALOG_WRITE_FREQ);
  analogWriteFrequency(KNEE_PWM_1, ANALOG_WRITE_FREQ);
  close_valves();
  Serial.begin(115200);
}

void loop() {
  char cmd;
  // h, H, k, K, t, T, e, d, c
  if (Serial.available() > 0) {
    // read two bytes
    cmd = Serial.read();
    close_valves();
    switch (cmd) {
      case 'h':
        Serial.println(F("HIP0"));
        analogWrite(HIP_PWM_0, PWM_VALUE);
        break;
      case 'H':
        Serial.println(F("HIP1"));
        analogWrite(HIP_PWM_1, PWM_VALUE);
        break;
      case 'k':
        Serial.println(F("KNE0"));
        analogWrite(KNEE_PWM_0, PWM_VALUE);
        break;
      case 'K':
        Serial.println(F("KNE1"));
        analogWrite(KNEE_PWM_1, PWM_VALUE);
        break;
      case 't':
        Serial.println(F("THI0"));
        analogWrite(THIGH_PWM_0, PWM_VALUE);
        break;
      case 'T':
        Serial.println(F("THI1"));
        analogWrite(THIGH_PWM_1, PWM_VALUE);
        break;
      case 'e':
        Serial.println(F("enabling motors"));
        enable_motors();
        break;
      case 'd':
        Serial.println(F("disabling motors"));
        disable_motors();
        break;
      case 'c':
        Serial.println(F("closing valves"));
        close_valves();
        break;
      default:
        Serial.print(F("Unknown command "));
        Serial.println(cmd);
        Serial.println(F("h or H to toggle hip pins"));
        Serial.println(F("t or T to toggle thigh pins"));
        Serial.println(F("k or K to toggle knee pins"));
        Serial.println(F("c to close/stop all pins"));
        Serial.println(F("e to enable, d to disable pins"));
        break;
    }
  }
}
