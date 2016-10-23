#define M1_EN_PIN 8
#define M2_EN_PIN 2

#define HIP_SENSOR A6
#define THIGH_SENSOR A3
#define KNEE_SENSOR A1
#define CALF_SENSOR A2

#define ANALOG_READ_RES 13

#define HIP_SENSOR_MIN 744
#define HIP_SENSOR_MAX 5776
#define HIP_CYILNDER_TRAVEL 8
#define HIP_SENSOR_UNITS_PER_INCH 629.0
#define HIP_CYLINDER_MIN_LENGTH 16
#define HIP_CYLINDER_MAX_LENGTH 24
#define HIP_RESTING_ANGLE 1.485512779977838

#define THIGH_SENSOR_MIN 272
#define THIGH_SENSOR_MAX 7336
#define THIGH_CYILNDER_TRAVEL 14
#define THIGH_SENSOR_UNITS_PER_INCH 504.57142857142856
#define THIGH_CYLINDER_MIN_LENGTH 24
#define THIGH_CYLINDER_MAX_LENGTH 38
#define THIGH_RESTING_ANGLE 0.33189216561617446

#define KNEE_SENSOR_MIN 1184
#define KNEE_SENSOR_MAX 7472
#define KNEE_CYILNDER_TRAVEL 12
#define KNEE_SENSOR_UNITS_PER_INCH 524.0
#define KNEE_CYLINDER_MIN_LENGTH 20
#define KNEE_CYLINDER_MAX_LENGTH 32
#define KNEE_RESTING_ANGLE 2.541326289554743

#define HIP_B 6.83905
//#define HIP_C 19.62051 // middle legs
#define HIP_C 19.16327 // front/back legs
#define THIGH_B 10.21631
#define THIGH_C 33.43093
#define KNEE_B 7.4386
#define KNEE_C 25.6021

#define N_SENSORS 4
int sensors[] = {KNEE_SENSOR, THIGH_SENSOR, HIP_SENSOR, CALF_SENSOR};


float hip_sensor_to_cylinder_length(int sensor) {
  if (sensor <= HIP_SENSOR_MIN) return HIP_CYLINDER_MIN_LENGTH;
  if (sensor >= HIP_SENSOR_MAX) return HIP_CYLINDER_MAX_LENGTH;
  return (sensor - HIP_SENSOR_MIN) / HIP_SENSOR_UNITS_PER_INCH + HIP_CYLINDER_MIN_LENGTH;
}

float thigh_sensor_to_cylinder_length(int sensor) {
  if (sensor <= THIGH_SENSOR_MIN) return THIGH_CYLINDER_MIN_LENGTH;
  if (sensor >= THIGH_SENSOR_MAX) return THIGH_CYLINDER_MAX_LENGTH;
  return (sensor - THIGH_SENSOR_MIN) / THIGH_SENSOR_UNITS_PER_INCH + THIGH_CYLINDER_MIN_LENGTH;
}

float knee_sensor_to_cylinder_length(int sensor) {
  if (sensor <= KNEE_SENSOR_MIN) return KNEE_CYLINDER_MIN_LENGTH;
  if (sensor >= KNEE_SENSOR_MAX) return KNEE_CYLINDER_MAX_LENGTH;
  return KNEE_CYLINDER_MAX_LENGTH - (KNEE_SENSOR_MAX - sensor) / KNEE_SENSOR_UNITS_PER_INCH;
  // if (sensor >= sensor_max) return max_length;
  // TODO check under min
  // return max_length - (sensor_max - sensor) / scalar;
}

float cylinder_length_to_angle(float cylinder_length, float edge_b, float edge_c) {
  // TODO optimize this by pre-computing sq(b) + sq(c) and 2bc
  return acos((-sq(cylinder_length) + sq(edge_b) + sq(edge_c)) / (2 * edge_b * edge_c));
}


void setup() {
  // disable motor outputs
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  // setup same analog read res
  analogReadResolution(ANALOG_READ_RES);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int value;
  float len;
  float ang;
  for (byte i=0; i<N_SENSORS; i++) {
    value = analogRead(sensors[i]);
    switch (i) {
      case 0:
        Serial.print("KNE ");
        len = knee_sensor_to_cylinder_length(value);
        ang = cylinder_length_to_angle(len, KNEE_B, KNEE_C);
        break;
      case 1:
        Serial.print("THI ");
        len = thigh_sensor_to_cylinder_length(value);
        ang = cylinder_length_to_angle(len, THIGH_B, THIGH_C);
        break;
      case 2:
        Serial.print("HIP ");
        len = hip_sensor_to_cylinder_length(value);
        ang = cylinder_length_to_angle(len, HIP_B, HIP_C);
        break;
      case 3:
        Serial.print("COM ");
        len = value;
        ang = len;
        break;
    };
    Serial.print(value, DEC);
    Serial.print(" ");
    Serial.print(len, DEC);
    Serial.print(" ");
    Serial.println(ang, DEC);
  }
  Serial.println("------------");
  delay(500);
}
