#include <comando.h>

// <optional>
// (type name, ...)
#define CMD_JOINTS 0  // -> (ul t, f hip, f thigh, f knee, f calf)
#define CMD_ESTOP 1 // <byte severity> -> <byte severity>
#define CMD_ENABLE 2 // -> <bool enable>
#define CMD_HEARTBEAT 3 // ->

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
}

void loop() {
  read_sensors();
  send_sensors();
  check_heartbeat();
  // update movements
  com.handle_stream();
  delay(10);  // something reasonable
}
