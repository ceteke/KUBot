import pyttsx
from Queue import Queue
import threading

class VoiceAssistant(threading.Thread):
    def __init__(self):
        super(VoiceAssistant, self).__init__()
        self.engine = pyttsx.init()
        self.q = Queue()
        self.daemon = True

    def add_say(self, msg):
        self.q.put(msg)

    def run(self):
        while True:
            self.engine.say(self.q.get())
            self.engine.runAndWait()
            self.q.task_done()
