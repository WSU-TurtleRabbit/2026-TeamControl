from TeamControl.network.robot_command import RobotCommand
from TeamControl.robot.constants import CMD_RUNTIME

class CommandManager:
    def __init__(self, isYellow, robot_id, dispatcher_q):
        self.isYellow = isYellow
        self.robot_id = robot_id
        self.output_queue = dispatcher_q
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.dribble = False
        self.kick = False
        self.run_time = CMD_RUNTIME  # failsafe command-hold expiry (s)

    def __repr__(self):
        return str(self._to_command())

    def _to_command(self):
        return RobotCommand(
            self.robot_id,
            self.vx,
            self.vy,
            self.w,
            self.kick,
            self.dribble,
            self.isYellow,
        )

    def update_command(self, **kwargs):
        self.vx = kwargs.get("vx", self.vx)
        self.vy = kwargs.get("vy", self.vy)
        self.w = kwargs.get("w", self.w)
        self.dribble = kwargs.get("dribble", self.dribble)
        self.kick = kwargs.get("kick", self.kick)

    def pack_and_send(self):
        command = self._to_command()
        if not self.output_queue.full():
            self.output_queue.put([command, self.run_time])
            # print("command sent", command)
            return True
        return False


if __name__ == "__main__":
    from queue import Queue

    dispatcher_q = Queue()
    cmd_mgr = CommandManager(isYellow=True, robot_id=1, dispatcher_q=dispatcher_q)
    cmd_mgr.update_command(vx=0.0, vy=0.0, w=0.0, dribble=False, kick=False)
    cmd_mgr.update_command(dribble=True, kick=False)
    is_sent = cmd_mgr.pack_and_send()
    if is_sent:
        print("Command sent : ", cmd_mgr)
