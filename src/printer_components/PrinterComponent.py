import abc
import threading
from queue import Queue, Empty
from typing import Optional, NamedTuple

from src.gcode.GCmd import GCmd


class CommandTask(NamedTuple):
    cmd: GCmd
    b: Optional[threading.Barrier]


class PrinterComponent(metaclass=abc.ABCMeta):
    """
    Defines the interface that all printer components need to provide.
    """

    def __init__(self, thread_poll_delay=0.01, name: str = 'Unnamed'):
        # Parameters
        self.queue_delay = thread_poll_delay
        self.name = name
        self.t: Optional[threading.Thread] = None
        self.send_q = Queue()
        self.recv_q = Queue()
        self.alive = threading.Event()

    def boot(self):
        self.t = threading.Thread(target=self.execute_task_loop, name=f'Task Handler ({self.name})')
        self.alive.set()
        self.t.start()

        self.hook_boot()

    def shutdown(self):
        if self.alive.isSet():
            # Put close object
            self.send_q.put(None)
            # Wait for send queue to finish, not all messages need to be received
            self.send_q.join()

        # Wait for task to finish, this can be done multiple times
        self.alive.clear()
        self.t.join()

        self.hook_shutdown()

    @abc.abstractmethod
    def hook_boot(self, *args, **kwargs):
        raise NotImplementedError

    @abc.abstractmethod
    def hook_shutdown(self, *args, **kwargs):
        raise NotImplementedError

    def assign_task(self, gcode: GCmd, barrier: Optional[threading.Barrier] = None):
        """
        Queue an incoming command.
        :param gcode: G-Code object
        :param barrier: Optional synchronization primitve. If this is passed the components are synchronized.
        :return:
        """
        task = CommandTask(gcode, barrier)
        self.send_q.put(task)

    def get_result(self):
        pass

    def execute_task_loop(self) -> None:
        """
        Continuous loop to handle queued commands in sync with other components.
        :return:
        """
        # Event to indicate that the thread should terminate
        while self.alive.isSet():
            # Get an item from the outgoing queue, timeout ensures that the loop condition can be checked
            try:
                task: Optional[CommandTask] = self.send_q.get(timeout=self.queue_delay)
            except Empty:
                # No task obtained, retry queue
                continue
            else:
                if task is None:
                    # End of queue
                    self.send_q.task_done()
                    break
                else:
                    # Component-specific command handling
                    print(f'{self.name} is handling {task.cmd}')
                    self.hook_handle_gcode(task.cmd, task.b)
                    print(f'{self.name} is done with {task.cmd}')
                    self.send_q.task_done()

    @abc.abstractmethod
    def hook_handle_gcode(self, gcode: GCmd, barrier: Optional[threading.Barrier]) -> None:
        """
        Implements the G-Code execution (needs to be overridden).
        :param gcode: G-Code object
        :param barrier: Synchronization primitive
        :return:
        """
        raise NotImplementedError

    def __enter__(self):
        self.boot()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
