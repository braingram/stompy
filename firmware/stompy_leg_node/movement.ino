// if movements are allowed
//bool movement_enable = false;

void on_enable(CommandProtocol *cmd){
  bool enable = true;
  if (cmd->has_arg()) {
    enable = cmd->get_arg<bool>();
  }
  on_enable(enable);
}

void on_enable(bool enable) {
  //movement_enable = enable;
  enable_node = enable;
  // also set heartbeat time so we don't immediately disable
  if (enable) set_heartbeat();
}
