#define STATUS_PIN 13

#define M1_EN_PIN 8 // 1
#define M2_EN_PIN 2

#define TEST_PWMS

#define HIP_PWM_0 9
#define HIP_PWM_1 10
#define THIGH_PWM_0 3
#define THIGH_PWM_1 4
#define KNEE_PWM_0 5
#define KNEE_PWM_1 6

#define HIP_SENSOR A6
#define THIGH_SENSOR A3 // A4
#define KNEE_SENSOR A1
#define CALF_SENSOR A2  // TODO ?

#define N_PWMS 6

int pwms[] = {THIGH_PWM_0, THIGH_PWM_1, KNEE_PWM_0, KNEE_PWM_1,HIP_PWM_0, HIP_PWM_1};

#define N_SENSORS 4
int sensors[] = {KNEE_SENSOR, THIGH_SENSOR, HIP_SENSOR, CALF_SENSOR};

int baseline[] = {0, 0, 0, 0};
int light_on[] = {0, 0, 0, 0};

void setup() {
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  for (int i=0; i<N_PWMS; i++) {
    pinMode(pwms[i], OUTPUT);
    analogWrite(pwms[i], 0);
  }
}

void print_sensors() {
  int value = 0;
  for (int i=0; i<N_SENSORS; i++) {
    value = analogRead(sensors[i]);
    Serial.print("Sensor[");
    Serial.print(i, DEC);
    Serial.print(",");
    Serial.print(baseline[i]);
    Serial.print("]");
    Serial.print(value, DEC);
    Serial.print(" --- ");
    Serial.print(value - baseline[i], DEC);
    if (value - baseline[i] > 200) {
      Serial.println(" HIGH");
    } else {
      Serial.println();
    }
  }
}

void loop() {
  digitalWrite(M1_EN_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);
  Serial.println("Motors enabled!");
  // read baseline light levels
  for (int i=0; i<N_SENSORS; i++) {
    baseline[i] = analogRead(sensors[i]);
  }
  print_sensors();
  delay(500);
  for (int i=0; i<N_PWMS; i++) {
    // turn on led
    analogWrite(pwms[i], 255);
    Serial.print("\tPWM ");
    Serial.print(i, DEC);
    Serial.println(" on!");
    // wait
    delay(1000);
    // read sensors
    print_sensors();
    // wait
    delay(1000);
    // turn off led
    analogWrite(pwms[i], 0);
    Serial.print("\tPWM ");
    Serial.print(i, DEC);
    Serial.println(" off!");
  }
  // disabling motors
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  Serial.println("Done!");
  print_sensors();
  Serial.print("\n\n\n\n\n");
  delay(5000);
  //while(1);
}
