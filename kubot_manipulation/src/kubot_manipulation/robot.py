from arm import Arm
from hand import Hand
from eye import Eye

class Robot:

    def __init__(self):
        self.run_id = self.get_run_id()
        self.arm = Arm()
        self.hand = Hand()
        self.eye = Eye(self.run_id)

    def get_run_id(self):
        with open('/home/cem/run_id.txt', 'r+') as f:
            lines = f.readlines()
            run_id = int(lines[0])
            new_run_id = run_id + 1
            f.seek(0)
            f.truncate()
            f.write(str(new_run_id))
            f.close()
            return run_id
