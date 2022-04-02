import actionlib
from abc import ABCMeta, abstractmethod


class AbstractActionServer(object):
    __metaclass__ = ABCMeta

    def __init__(self, action_name, action_spec):
        self.name = action_name
        self.stopping = False
        self.is_executing = False
        self.end_callback = None
        self.server = actionlib.SimpleActionServer(action_name, action_spec, self._execute, False)

    def start(self):
        self.server.start()

    def stop(self, callback=None):
        if self.is_executing:
            self.stopping = True
            self.end_callback = callback
        else:
            self.server.stop()
            if callback is None: return
            callback()

    def _execute(self, goal):
        self.is_executing = True
        self.execute(goal)
        self.is_executing = False
        if self.stopping:
            self.server.stop()
            if self.end_callback is None: return
            self.end_callback()

    @abstractmethod
    def execute(self, goal):
        pass
