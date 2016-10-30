#define M1_EN_PIN 8
#define M2_EN_PIN 2

#define HIP_PWM_E 9  // extend
#define HIP_PWM_R 10  // retract
#define THIGH_PWM_E 3  // extend
#define THIGH_PWM_R 4  // retract
#define KNEE_PWM_E 5  // extend
#define KNEE_PWM_R 6  // retract

#define HIP_SENSOR A6
#define THIGH_SENSOR A3
#define KNEE_SENSOR A1
#define CALF_SENSOR A2

#define ANALOG_READ_RES 16
#define ANALOG_WRITE_RES 10
#define ANALOG_WRITE_FREQ 20000.0  // to make current feedback happy

#define INPUT_TIMEOUT 500UL  // ms
#define SENSOR_REPORT_PERIOD 100UL  // ms

unsigned long last_input_time = 0;
unsigned long last_sensor_report_time = 0;

char joint = '\x00';
char pwm_input_buffer[6];
byte pwm_buffer_index = 0;

//#define REPORT_PWMS

void enable_motors() {
  digitalWrite(M1_EN_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);
}

void disable_motors() {
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
}

void set_hip_pwm(int pwm) {
  if (pwm > 0) {
    analogWrite(HIP_PWM_R, 0);
    analogWrite(HIP_PWM_E, pwm);
  } else {
    analogWrite(HIP_PWM_E, 0);
    analogWrite(HIP_PWM_R, -pwm);
  }
#ifdef REPORT_PWMS
  Serial.print("h");
  Serial.println(pwm);
#endif
}

void set_thigh_pwm(int pwm) {
  if (pwm > 0) {
    analogWrite(THIGH_PWM_R, 0);
    analogWrite(THIGH_PWM_E, pwm);
  } else {
    analogWrite(THIGH_PWM_E, 0);
    analogWrite(THIGH_PWM_R, -pwm);
  }
#ifdef REPORT_PWMS
  Serial.print("h");
  Serial.println(pwm);
#endif
}

void set_knee_pwm(int pwm) {
  if (pwm > 0) {
    analogWrite(KNEE_PWM_R, 0);
    analogWrite(KNEE_PWM_E, pwm);
  } else {
    analogWrite(KNEE_PWM_E, 0);
    analogWrite(KNEE_PWM_R, -pwm);
  }
#ifdef REPORT_PWMS
  Serial.print("h");
  Serial.println(pwm);
#endif
}

void close_valves() {
  set_hip_pwm(0);
  set_thigh_pwm(0);
  set_knee_pwm(0);
}

void report_sensors() {
  Serial.print("H");
  Serial.println(analogRead(HIP_SENSOR), DEC);
  Serial.print("T");
  Serial.println(analogRead(THIGH_SENSOR), DEC);
  Serial.print("K");
  Serial.println(analogRead(KNEE_SENSOR), DEC);
  Serial.print("C");
  Serial.println(analogRead(CALF_SENSOR), DEC);
}

int check_input() {
  while (Serial.available()) {
     char c = Serial.read();
     if ((c == '\n') | (c == '\r')) {
      pwm_input_buffer[pwm_buffer_index] = '\x00';
      return pwm_buffer_index;
     }
     if (joint == '\x00') {
      joint = c;
     } else {
      pwm_input_buffer[pwm_buffer_index] = c;
      pwm_buffer_index += 1;
     }
  }
  return 0;
}

void reset_input() {
  pwm_buffer_index = 0;
  joint = '\x00';
}

void setup() {
  // disable motor outputs
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  disable_motors();
  pinMode(HIP_PWM_R, OUTPUT);
  pinMode(HIP_PWM_E, OUTPUT);
  pinMode(THIGH_PWM_R, OUTPUT);
  pinMode(THIGH_PWM_E, OUTPUT);
  pinMode(KNEE_PWM_R, OUTPUT);
  pinMode(KNEE_PWM_E, OUTPUT);
  analogWriteResolution(ANALOG_WRITE_RES);
  analogWriteFrequency(HIP_PWM_E, ANALOG_WRITE_FREQ);
  analogWriteFrequency(HIP_PWM_R, ANALOG_WRITE_FREQ);
  analogWriteFrequency(THIGH_PWM_E, ANALOG_WRITE_FREQ);
  analogWriteFrequency(THIGH_PWM_R, ANALOG_WRITE_FREQ);
  analogWriteFrequency(KNEE_PWM_E, ANALOG_WRITE_FREQ);
  analogWriteFrequency(KNEE_PWM_R, ANALOG_WRITE_FREQ);
  close_valves();
  // setup analog read res
  analogReadResolution(ANALOG_READ_RES);
  Serial.begin(115200);
}

void loop() {
  if (check_input()) {
    int pwm = atoi(pwm_input_buffer);
    enable_motors();
    switch (joint) {
      case 'H':
        set_hip_pwm(pwm);
        break;
      case 'T':
        set_thigh_pwm(pwm);
        break;
      case 'K':
        set_knee_pwm(pwm);
        break;
    }
    reset_input();
    last_input_time = millis();
  };
  if ((millis() - last_input_time) > INPUT_TIMEOUT) {
    close_valves();
    disable_motors();
  };
  if ((millis() - last_sensor_report_time) > SENSOR_REPORT_PERIOD) {
    report_sensors();
    last_sensor_report_time = millis();
  }
}
