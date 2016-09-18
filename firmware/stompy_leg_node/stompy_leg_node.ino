#include <comando.h>


#define FAKE_JOINTS


// ---------- pins -------------
#define STATUS_PIN 13

#define HIP_PWM_0 9
#define HIP_PWM_1 10
#define THIGH_PWM_0 3
#define THIGH_PWM_1 4
#define KNEE_PWM_0 5
#define KNEE_PWM_1 6

// joint sensor pins
#define HIP_SENSOR A6
#define THIGH_SENSOR A4
#define KNEE_SENSOR A3
#define CALF_SENSOR A2  // TODO ?
// ---------- pins -------------


// --------- commands ----------
// <optional>
// (type name, ...)
#define CMD_JOINTS 0  // -> (ul t, f hip, f thigh, f knee, f calf)
#define CMD_ESTOP 1 // <byte severity> -> <byte severity>
#define CMD_ENABLE 2 // -> <bool enable>
#define CMD_HEARTBEAT 3 // ->
#define CMD_STATUS 4 // -> <bool enable>
#define CMD_GET_TIME 5 // -> <unsigned long teensy_time>

// -> <byte id, unsigned long time, float hip, float thigh, float knee>
// <byte index> <-
#define CMD_NEW_POINT 6
#define CMD_DROP_POINT 7 // -> <byte index>
#define CMD_POINT_REACHED 8 // <byte id> <-
#define CMD_DONE_MOVING 9
// --------- commands ----------

// setup analog write as per:
// https://www.pjrc.com/teensy/td_pulse.html
// for a 72 MHz CPU, 16 bit analog write
// pwm freq = 549.3164 Hz
#ifdef FAKE_JOINTS
#define ANALOG_WRITE_FREQ 50
#else
#define ANALOG_WRITE_FREQ 1098.632
#endif
#define ANALOG_WRITE_RES 15
#define ANALOG_READ_RES 16

// assuming 100 milliseconds between points, this should give
// ~5 seconds of buffering
#define BUFFER_LENGTH 50

bool enable_node = false;

unsigned long last_sensor_time = 0;
float hip_angle = 0.0;
float thigh_angle = 0.0;
float knee_angle = 0.0;
float calf_angle = 0.0;

Comando com = Comando(Serial);
TextProtocol text = TextProtocol(com);
CommandProtocol cmd = CommandProtocol(com);

void setup() {
  // setup pins
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  // setup valve pins
  analogWriteResolution(ANALOG_WRITE_RES);
  analogWriteFrequency(HIP_PWM_0, ANALOG_WRITE_FREQ);
  analogWriteFrequency(HIP_PWM_1, ANALOG_WRITE_FREQ);
  analogWriteFrequency(THIGH_PWM_0, ANALOG_WRITE_FREQ);
  analogWriteFrequency(THIGH_PWM_1, ANALOG_WRITE_FREQ);
  analogWriteFrequency(KNEE_PWM_0, ANALOG_WRITE_FREQ);
  analogWriteFrequency(KNEE_PWM_1, ANALOG_WRITE_FREQ);
  // setup sensor pins
  analogReadResolution(ANALOG_READ_RES);

  Serial.begin(115200);

  com.register_protocol(0, text);
  com.register_protocol(1, cmd);

  cmd.register_callback(CMD_ESTOP, on_estop);
  cmd.register_callback(CMD_ENABLE, on_enable);
  cmd.register_callback(CMD_HEARTBEAT, on_heartbeat);
  cmd.register_callback(CMD_STATUS, on_status);
  cmd.register_callback(CMD_GET_TIME, get_time);

  cmd.register_callback(CMD_NEW_POINT, on_new_point);
  cmd.register_callback(CMD_DROP_POINT, on_drop_point);
  clear_buffers();
}

void loop() {
  com.handle_stream();
  if (enable_node) {
    check_heartbeat();
    read_sensors();
    send_sensors();
    update_movement();
  } else {
    digitalWrite(STATUS_PIN, LOW);
    close_all_valves();
    clear_buffers();
  }
  //delay(10);  // something reasonable
}

void on_status(CommandProtocol *cmd) {
  cmd->start_command(CMD_STATUS);
  cmd->add_arg(enable_node);
  cmd->finish_command();
}
