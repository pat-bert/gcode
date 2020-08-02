import abc
import logging
import threading
from queue import Queue, Empty
from typing import Optional

from src.clients.IClient import IClient, Msg, ClientError


class ThreadedClient(IClient):
    """
    A template for a threaded client. Behavior can be modified by overriding hooks.

    Inheriting classes MUST implement client-specific methods for:
    - connecting
    - closing
    - taking a message and returning the related response (one method)

    Optional hooks can be implemented to:
    - pre-process messages before queueing them for sending
    - post-process responses before returning them to the user
    - Name the communication thread for debugging
    - execute custom logic just before connecting

    If all methods are implemented these features are provided:
    - Running the message handling in a separate thread
    - Thread-communication via thread-safe queues
    - Graceful thread termination via a thread-safe flag
    - Optional message and response logging
    - Connected status

    """

    def __init__(self, thread_poll_delay=0.01, kind=''):
        """
        Creates everything required for a threaded client (threads, queues, thread-safe flags).
        :param thread_poll_delay: Time in seconds to block while getting items from the queue.
        """
        # Parameters
        self.queue_delay = thread_poll_delay
        self.kind = kind

        # Queues and thread for communication from and to worker tread
        self.t: Optional[threading.Thread] = None
        self.send_q = Queue()
        self.recv_q = Queue()
        self.alive = threading.Event()

    def connect(self) -> None:
        """
        Connect the client and launch a worker thread.
        :raises: Propagates all exceptions.
        :return: None
        """
        # Pre-connect procedure
        self.hook_pre_connect()
        thread_name = self.hook_thread_name()

        # Attemp to open a connection
        peer_name = self.hook_connect()

        if peer_name is not None:
            logging.info(f'Connected to {peer_name}.')
        else:
            logging.info('Connected.')

        # Post-connect procedure (read initial message)
        self.hook_post_successful_connect()

        # Start thread
        self.alive.set()

        # Just in case..
        if thread_name is not None:
            self.t = threading.Thread(target=self.mainloop, name=thread_name)
        else:
            self.t = threading.Thread(target=self.mainloop)

        self.t.start()

    def mainloop(self) -> None:
        """ Main loop function for the worker thread for the network communication.
        Regularly polls the alive event to terminate gracefully if required.

        Handles outgoing messages put into the send queue via self.send().

        Message handling is implemented by self.hook_handle_msg():
        - Client-specific sending of the message and receiving of the related response
        - If the returned response has zero length it is interpreted as connection closed by the other end

        Incoming Msg-objects are interpreted as follows:
        - [0] None messages cause the thread to terminate
        - [1] Flag to specify whether the outgoing message should be logged
        - [2] Flag to specify whether the incoming response should be logged

        Responses are put into the receive queue and can be retrieved via self.receive().

        :return: None
        :raises: ServerClosedConnectionError if an empty string is received
        """
        # Event to indicate that the thread should terminate
        while self.alive.isSet():
            # Get an item from the outgoing queue, timeout ensures that the loop condition can be checked
            try:
                msg = self.send_q.get(timeout=self.queue_delay)
            except Empty:
                continue
            else:
                if msg is None:
                    # End of queue
                    self.send_q.task_done()
                    break
                else:
                    # Regular message
                    msg, silent_send, silent_recv = msg.unpack()

                    if not silent_send:
                        logging.debug(f'{self.kind} >>: {str(msg).strip()}')

                    # Client-specific message handling
                    response = self.hook_handle_msg(msg)

                    if not silent_recv:
                        logging.debug(f'{self.kind} <<: {str(response).strip()}')

                    # Put the response and indicate that the task is done
                    self.recv_q.put(response)
                    self.send_q.task_done()

                    # Server closed down connection
                    if response is not None and len(response) == 0:
                        self.alive.clear()
                        return

    def close(self) -> None:
        """
        Close the client cleanly.
        :return: None
        """
        if self.is_connected:
            # Put close object
            self.send_q.put(None)

            # Wait for send queue to finish, not all messages need to be received
            self.send_q.join()

            # Wait for task to finish, this can be done multiple times
            self.alive.clear()
            self.t.join()

            # Emptying queue
            if not self.recv_q.empty():
                logging.warning('Not all responses were received:')
                logging.warning('============================')
                for i in range(self.recv_q.unfinished_tasks):
                    try:
                        response = self.recv_q.get_nowait()
                    except Empty:
                        logging.info('Queue empty.')
                    else:
                        logging.warning(f'{i + 1}.) response:\n{response}')
                        logging.warning('============================')
                        self.recv_q.task_done()

            # Client specific closing
            self.hook_close()
            logging.info('Closed communication.')
        else:
            logging.info('Communication already closed.')

    def send(self, msg: Optional[str], silent_send: bool = False, silent_recv: bool = False) -> None:
        """
        Sends a message via the worker thread for the protocol communication.
        :param msg: Message to be sent. If message is None instead of a string the queue is signaled to be done.
        :param silent_send: Flag to specify whether the outgoing message should be logged.
        :param silent_recv: Flag to specify whether the incoming response should be logged.
        :return: None
        :raises: ClientError if the client is not connected
        """
        # Put the message to the outgoing queue of the protocol, None is used to end the communication
        if self.is_connected:
            if msg is not None:
                # Preprocess and pack
                processed_msg = self.hook_pre_send(msg)
                packed_message = Msg(processed_msg, silent_send, silent_recv)
                self.send_q.put(packed_message)
            else:
                self.send_q.put(None)
            return
        raise ClientError('Client needs to be connected before sending since this could lead to unexpected behavior.')

    def wait_send(self, msg: str) -> None:
        """
        Sends a message and waits until all messages are processed.
        :param msg: Message to be sent.
        :return: None
        """
        self.send(msg)
        self.send_q.join()

    def receive(self, silence_errors=False) -> str:
        """
        Get the last response received by the worker thread.
        :param silence_errors: Specify whether exceptions should be silenced.
        :return: Message string without status code
        :raises: ClientError if the client is not connected
        """
        # Thread needs to be running or this will block indefinetly
        if self.is_connected:
            # Get the last response from the queue
            response = self.recv_q.get()
            self.recv_q.task_done()
            # Call hook
            return self.hook_post_receive(response, silence_errors)
        raise ClientError('Client needs to be connected before sending since this could lead to unexpected behavior.')

    @property
    def is_connected(self) -> bool:
        """
        Checks if the alive event is set.
        :return: Flag to indicate that the alive event is set.
        """
        return self.alive.isSet()

    @abc.abstractmethod
    def hook_connect(self) -> Optional[str]:
        """
        Client-specific connection. Needs to be overriden.
        :return: Name of the peer
        """

    @abc.abstractmethod
    def hook_close(self) -> None:
        """
        Client-specific closing. Needs to be overriden.
        :return: None
        """

    @abc.abstractmethod
    def hook_handle_msg(self, msg: str) -> str:
        """
        Client-specific hook for handling messages within the worker thread. Needs to be overriden.
        :param msg: Message string to be sent
        :return: Response string received
        """

    @staticmethod
    def hook_thread_name() -> Optional[str]:
        """
        Client-specific thread-naming. Can be overriden. Defaults to standard thread naming.
        :return: None
        """

    @staticmethod
    def hook_pre_connect() -> None:
        """
        Client-specific hook to be executed right before connecting. Can be overriden.
        :return: None
        """

    @staticmethod
    def hook_post_successful_connect() -> None:
        """
        Client-specific hook to be executed right after connecting. Can be overriden.
        :return: None
        """

    @staticmethod
    def hook_pre_send(msg: str) -> str:
        """
        Client-specific hook for pre-processing outgoing messages from the user. Can be overriden.
        :param msg: Message to be sent.
        :return: Modified response string
        """
        return msg

    @staticmethod
    def hook_post_receive(response: str, silence_errors: bool) -> str:
        """
        Client-specific hook for handling incoming messages queued by the worker thread. Can be overriden.
        :return: Processed response string
        """
        return response
