/*
 * Used for precision time protocol (ptp) time syncing
 * 
 * Steps:
 *   1) master notes time: t1, requests slave time: t1p (t1p sent to master)
 *   2) requests slave time: t2, on receipt, master notes time t2p
 *   3) master computer slave offset:
 *     offset = (t1p - t1 - t2p + t2) / 2
 * 
 * to convert from master to slave time:
 *   master_time + offset  
 */

void get_time(CommandProtocol *cmd) {
  cmd->start_command(CMD_GET_TIME);
  cmd->add_arg(millis());
  cmd->finish_command();
}

