// joint sensor pins
#define HIP_SENSOR 20
#define THIGH_SENSOR 18
#define KNEE_SENSOR 17
#define CALF_SENSOR A3  // ?

#define SENSOR_PERIOD 100

/* sensor to cylinder length
 */
#define HIP_SENSOR_MIN 744
//#define HIP_SENSOR_MAX 5776
//#define HIP_CYILNDER_TRAVEL 8
#define HIP_SENSOR_UNITS_PER_INCH 629.0
#define HIP_CYLINDER_MIN_LENGTH 16
#define HIP_RESTING_ANGLE 1.485512779977838

#define THIGH_SENSOR_MIN 272
//#define THIGH_SENSOR_MAX 7336
//#define THIGH_CYILNDER_TRAVEL 14
#define THIGH_SENSOR_UNITS_PER_INCH 504.57142857142856
#define THIGH_CYLINDER_MIN_LENGTH 24
#define THIGH_RESTING_ANGLE 0.33189216561617446

#define KNEE_SENSOR_MIN 1184
//#define KNEE_SENSOR_MAX 7472
//#define KNEE_CYILNDER_TRAVEL 12
#define KNEE_SENSOR_UNITS_PER_INCH 524.0
#define KNEE_CYLINDER_MIN_LENGTH 20
#define KNEE_RESTING_ANGLE 2.541326289554743

/* cylinder length to joint angles
 *   a = cylinder length
 *   b = rod end to joint pivot
 *   c = joint pivot to cap end
 *   A = angle between b and c, angle at joint pivot
 *
 *   cos(A) = (-a^2 + b^2 + c^2) / (2 * b * c)
 */
// TODO get these to a higher precision
#define HIP_B 6.83905
#define HIP_C 19.62051 // TODO doesn't match
//#define HIP_C 19.16 // TODO doesn't match
#define THIGH_B 10.21631
#define THIGH_C 33.43093
#define KNEE_B 7.4386
#define KNEE_C 25.6021

unsigned long last_sensor_publish_time = 0;
float sensor_to_cylinder_length(int sensor, int sensor_min, float scalar, int min_length) {
  return (sensor - sensor_min) / scalar + min_length;
}


float cylinder_length_to_angle(float cylinder_length, float edge_b, float edge_c) {
  // TODO optimize this by pre-computing sq(b) + sq(c) and 2bc
  return acos((-sq(cylinder_length) + sq(edge_b) + sq(edge_c)) / (2 * edge_b * edge_c));
}


void read_sensors() {
  hip_angle = cylinder_length_to_angle(sensor_to_cylinder_length(
      analogRead(HIP_SENSOR), HIP_SENSOR_MIN,
      HIP_SENSOR_UNITS_PER_INCH, HIP_CYLINDER_MIN_LENGTH),
        HIP_B, HIP_C);
  thigh_angle = cylinder_length_to_angle(sensor_to_cylinder_length(
      analogRead(THIGH_SENSOR), THIGH_SENSOR_MIN,
      THIGH_SENSOR_UNITS_PER_INCH, THIGH_CYLINDER_MIN_LENGTH),
        THIGH_B, THIGH_C);
  knee_angle = cylinder_length_to_angle(sensor_to_cylinder_length(
      analogRead(KNEE_SENSOR), KNEE_SENSOR_MIN,
      KNEE_SENSOR_UNITS_PER_INCH, KNEE_CYLINDER_MIN_LENGTH),
        KNEE_B, KNEE_C);

  thigh_angle = thigh_angle - THIGH_RESTING_ANGLE;
  knee_angle = KNEE_RESTING_ANGLE - knee_angle;
  // TODO work out hip resting angle
  hip_angle = hip_angle - HIP_RESTING_ANGLE;

  // TODO calf
  calf_angle = analogRead(CALF_SENSOR);
  last_sensor_time = millis();
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
