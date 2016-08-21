void send_estop() {
  send_estop(0);
}

void send_estop(byte severity) {
  cmd.start_command(CMD_ESTOP);
  cmd.add_arg(severity);
  cmd.finish_command();
}

void on_estop(CommandProtocol *cmd) {
  byte severity = 0;
  if (cmd->has_arg()) {
    severity = cmd->get_arg<byte>();
  }
  on_estop(severity);
}

void on_estop(byte severity) {
  on_enable(false);
  // TODO
}
