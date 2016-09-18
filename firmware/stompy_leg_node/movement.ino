// if movements are allowed
//bool movement_enable = false;

#define POINT_ENABLE 1
#define POINT_DISABLE 0

byte pids[BUFFER_LENGTH];
unsigned long point_times[BUFFER_LENGTH];
float hip_angles[BUFFER_LENGTH];
float thigh_angles[BUFFER_LENGTH];
float knee_angles[BUFFER_LENGTH];
byte point_states[BUFFER_LENGTH];

byte write_index = 0;
byte move_index = 0;
byte moving = 0;


float hip_angle_error = 0.;
float thigh_angle_error = 0.;
float knee_angle_error = 0.;


void clear_buffers() {
  for (write_index=0; write_index < BUFFER_LENGTH; write_index++) {
    point_states[write_index] = POINT_DISABLE;
  };
  write_index = 0;
};

void close_all_valves() {
  analogWrite(HIP_PWM_0, 0);
  analogWrite(HIP_PWM_1, 0);
  analogWrite(THIGH_PWM_0, 0);
  analogWrite(THIGH_PWM_1, 0);
  analogWrite(KNEE_PWM_0, 0);
  analogWrite(KNEE_PWM_1, 0);
}

void on_enable(bool enable) {
  //movement_enable = enable;
  enable_node = enable;
  // also set heartbeat time so we don't immediately disable
  if (enable) {
    set_heartbeat();
  } else {
    // shut down
    digitalWrite(STATUS_PIN, LOW);
    close_all_valves();
  };
}

void on_enable(CommandProtocol *cmd){
  bool enable = true;
  if (cmd->has_arg()) {
    enable = cmd->get_arg<bool>();
  }
  on_enable(enable);
}

// add a new point and return the index
void on_new_point(CommandProtocol *cmd) {
  if (!cmd->has_arg()) return;
  byte pid = cmd->get_arg<byte>();
  if (!cmd->has_arg()) return;
  unsigned long point_time = cmd->get_arg<unsigned long>();
  if (!cmd->has_arg()) return;
  float hip = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float thigh = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float knee = cmd->get_arg<float>();
  pids[write_index] = pid;
  point_times[write_index] = point_time;
  hip_angles[write_index] = hip;
  thigh_angles[write_index] = thigh;
  knee_angles[write_index] = knee;
  point_states[write_index] = POINT_ENABLE;
  // send back point index
  cmd->start_command(CMD_NEW_POINT);
  cmd->add_arg(write_index);
  cmd->finish_command();
  if (!moving) {
    move_index = write_index;
    moving = 1;
  };
  write_index++;
  if (write_index == BUFFER_LENGTH) write_index = 0;
}

// drop a point by index
void on_drop_point(CommandProtocol *cmd) {
  if (!cmd->has_arg()) return;
  byte index = cmd->get_arg<byte>();
  point_states[index] = POINT_DISABLE;
}


float compute_angle_error() {
  hip_angle_error = hip_angle - hip_angles[move_index];
  thigh_angle_error = thigh_angle - thigh_angles[move_index];
  knee_angle_error = knee_angle - knee_angles[move_index];
  return abs(hip_angle_error) + abs(thigh_angle_error) + abs(knee_angle_error);
}

// increment move_index and return 1 if a valid next point is found
byte advance_move_index() {
  byte new_index = move_index + 1;
  if (new_index == BUFFER_LENGTH) new_index = 0;
  while (new_index != move_index) {
    if (point_states[new_index] == POINT_ENABLE) {
      move_index = new_index;
      return 1;
    };
    new_index++;
    if (new_index == BUFFER_LENGTH) new_index = 0;
  };
  return 0;
};

void update_movement() {
  if (moving & enable_node) {
    // check if point is reached, if so
    if (compute_angle_error() < 0.01) {
      // TODO check time tolerance
      // report point reached
      cmd.start_command(CMD_POINT_REACHED);
      cmd.add_arg(move_index);
      cmd.finish_command();
      point_states[move_index] = POINT_DISABLE;
      // advance index to next point
      if (advance_move_index()) {
        // advance and recurse
        return update_movement();
      } else {
        // if no point, stop
        moving = 0;
        // report done moving
        cmd.start_command(CMD_DONE_MOVING);
        cmd.finish_command();
        return;
      }
    };
    // if not reached
    // TODO check tolerances (time and position), if outside tolerances
    //   - stop, report failure
    // control valves
#ifdef FAKE_JOINTS
    if (millis() >= point_times[move_index]) {
      hip_angle = hip_angles[move_index];
      // scale hip (+-, radians) to 15 bit (32768)
      analogWrite(HIP_PWM_0, hip_angle * 1159 + 2457);
      thigh_angle = thigh_angles[move_index];
      analogWrite(THIGH_PWM_0, thigh_angle * 1042 + 1638);
      knee_angle = knee_angles[move_index];
      analogWrite(KNEE_PWM_0, knee_angle * -690 + 1638);
    };
#endif
  };
};
