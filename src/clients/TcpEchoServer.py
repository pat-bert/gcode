import socket
import threading
from typing import Union


class TcpEchoServer:
    """
    A simple implementation of a TCP/IP-Server that echoes all messages.
    Only one connection is supported but waiting for a connection and handling a connection
    is handled in a separate thread each.
    """

    listen_thread: Union[None, threading.Thread]
    s: Union[None, socket.socket]

    def __init__(self, host, port):
        """
        Initialize the TCP Server
        :param host: Hostname as IPv4-adress
        :param port: TCP-Port to be listened to
        """
        # Socket properties
        self._is_listening = False
        self.s = None
        self.host = host
        self.port = port

        # Set the flag for communication threads
        self.listen_thread = None
        self.isAlive = threading.Event()

    def listen(self) -> None:
        """
        Starts the server in listening mode.
        :return: None
        """
        # Guard against repeated calls
        if not self._is_listening:
            # Set the flag for the threads to true
            self.isAlive.set()

            # Create a new socket (one-time use, will be closed on server shutdown)
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, socket.SHUT_RDWR)

            # Bind the adress
            self.s.bind((self.host, self.port))

            # Listen for connections
            print('Server listening.')
            self.s.listen(5)

            # Create a new thread for accepting incoming connections so that this function does not block
            self.listen_thread = threading.Thread(target=self._listening_loop, name='TCP-Accepter')
            self.listen_thread.start()

            # Change the status
            self._is_listening = True
        else:
            print('Server already listening.')

    @property
    def is_listening(self) -> bool:
        """
        Checks whether the server is listening
        :return: Flag that indicates the state
        """
        # Listening should not be manipulated directly to ensure the guard against repeated listening calls
        return self._is_listening

    def shutdown(self) -> None:
        """
        Signals the communication threads to shutdown.
        :return: None
        """
        print('Waiting for communication thread to shutdown.')
        self.isAlive.clear()
        self.listen_thread.join()
        # Reset the flag so that the same server can be reused for a new connection
        self._is_listening = False
        # Also close the server socket so that it can be rebound
        self.s.close()
        print('Shutting down server.')

    def _listening_loop(self) -> None:
        """
        Wait for a connection to start and to finish (handler thread target).
        :return: None
        """
        # Wait for a connection but fail fast
        self.s.settimeout(0.1)
        while self.isAlive.isSet():
            try:
                conn, client_addr = self.s.accept()
            except socket.timeout:
                continue
            else:
                print('New connection from {}.'.format(client_addr))

                # Setup the worker thread and wait until the communication is finished
                t = threading.Thread(target=self._echo_loop, args=(conn,), name='TCP-Echo ({}:{})'.format(*client_addr))
                t.start()
                t.join()

    def _echo_loop(self, connection: socket.socket) -> None:
        """
        Get a connection and echo all the data received on it back (worker thread target).
        :param connection: Socket to communicate through with the client
        :return: None
        """
        # Set socket to timeout to allow checking the alive flag
        connection.settimeout(0.1)

        # Loop for messages to send back
        while self.isAlive.isSet():
            try:
                # Fail fast
                data = connection.recv(256)
            except socket.timeout:
                # No message available, retry if still alive
                continue
            except socket.error as e:
                # Different exception: Shutdown connection
                print(e)
                connection.close()
            else:
                if data:
                    # Hook for resolving response
                    echo_msg = self.determine_response(data)
                    # Echo the message and wait for more if still alive
                    connection.sendall(echo_msg)
                else:
                    # Data is empty and there will be no future messages
                    break

        # Shutdown connection properly
        connection.close()

    @staticmethod
    def determine_response(msg: bytes) -> bytes:
        """
        Hook for implementing a dynamic response.
        :param msg:
        :return:
        """
        return msg


class ConfigurableEchoServer(TcpEchoServer):
    def __init__(self, host, port, encoding: str):
        super().__init__(host, port)
        self.encoding = encoding
        self.prefix = None
        self.postfix = None
        self.replace_msg = None

    def reconfigure(self, prefix: Union[None, str] = None,
                    postfix: Union[None, str] = None,
                    msg: Union[None, str] = None):

        # Set the prefix as bytes
        if prefix is not None:
            self.prefix = bytes(prefix, encoding=self.encoding)
        else:
            self.prefix = prefix

        # Set the postfix as bytes
        if postfix is not None:
            self.postfix = bytes(postfix, encoding=self.encoding)
        else:
            self.postfix = postfix

        if msg is not None:
            self.replace_msg = bytes(msg, encoding=self.encoding)
        else:
            self.replace_msg = msg

    def determine_response(self, msg: bytes) -> bytes:
        if self.replace_msg is not None:
            msg = self.replace_msg
        if self.prefix is not None:
            msg = self.prefix + msg
        if self.postfix is not None:
            msg = msg + self.postfix
        return msg
