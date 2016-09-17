#include <comando.h>

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


// assuming 100 milliseconds between points, this should give
// ~5 seconds of buffering
#define BUFFER_LENGTH 50

#define FAKE_JOINTS

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
  setup_buffers();
}

void loop() {
  // TODO convert this to a state machine
  com.handle_stream();
  if (enable_node) {
    check_heartbeat();
    read_sensors();
    send_sensors();
    update_movement();
  };
  //delay(10);  // something reasonable
}

void on_status(CommandProtocol *cmd) {
  cmd->start_command(CMD_STATUS);
  cmd->add_arg(enable_node);
  cmd->finish_command();
}
