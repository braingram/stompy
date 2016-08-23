// time of last heartbeat
unsigned long last_heartbeat = 0;

// maximum time between heartbeats, when exceeded, estop
#define MAX_HEARTBEAT 1000

void set_heartbeat() {
  last_heartbeat = millis();
}

void on_heartbeat(CommandProtocol *cmd) {
  last_heartbeat = millis();
  // immediately ping back
  cmd->send_command(CMD_HEARTBEAT);
}

void check_heartbeat() {
  unsigned long t = millis();
  if ((t - last_heartbeat) > MAX_HEARTBEAT) {
    on_enable(false);
    send_estop();
  }
}
