// joint sensor pins
#define HIP_SENSOR 20
#define THIGH_SENSOR 18
#define KNEE_SENSOR 17
#define CALF_SENSOR A3  // ?

#define SENSOR_PERIOD 100
unsigned long last_sensor_publish_time = 0;

// last sensor readings
unsigned long last_sensor_time = 0;
float hip_angle = 0.0;
float thigh_angle = 0.0;
float knee_angle = 0.0;
float calf_angle = 0.0;

void read_sensors() {
  hip_angle = analogRead(HIP_SENSOR);
  thigh_angle = analogRead(THIGH_SENSOR);
  knee_angle = analogRead(KNEE_SENSOR);
  calf_angle = analogRead(CALF_SENSOR);
  last_sensor_time = millis();
  // TODO convert?
}

void send_sensors() {
  if ((last_sensor_time - last_sensor_publish_time) > SENSOR_PERIOD) {
    cmd.start_command(CMD_JOINTS);
    cmd.add_arg(last_sensor_time);
    cmd.add_arg(hip_angle);
    cmd.add_arg(thigh_angle);
    cmd.add_arg(knee_angle);
    cmd.add_arg(calf_angle);
    cmd.finish_command();
    last_sensor_publish_time = last_sensor_time;
  };
}
