import socket
import threading


class TcpEchoServer:
    """
    A simple implementation of a TCP/IP-Server that echoes all messages.
    Only one connection is supported but waiting for a connection and handling a connection
    is handled in a separate thread each.
    """

    def __init__(self, host, port):
        """
        Initialize the TCP Server
        :param host: Hostname as IPv4-adress
        :param port: TCP-Port to be listened to
        """
        # Create the socket
        self.listen_thread = None
        self._is_listening = False
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the adress
        self.s.bind((host, port))

        # Set the flag for communication threads
        self.isAlive = threading.Event()
        self.isAlive.set()

    def listen(self) -> None:
        """
        Sets the server into listening mode.
        :return: None
        """
        # Guard against repeated calls
        if not self._is_listening:
            # Listen for connections
            print('Server listening.')
            self.s.listen(1)

            # Create a new thread
            self.listen_thread = threading.Thread(target=self.listening_loop, name='TCP-Accepter')
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

    def listening_loop(self) -> None:
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
                t = threading.Thread(target=self.echo_loop, args=(conn,), name='TCP-Echo ({}:{})'.format(*client_addr))
                t.start()
                t.join()

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
        print('Shutting down server.')

    def echo_loop(self, connection: socket.socket) -> None:
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
            except socket.error:
                # Different exception: Shutdown connection
                connection.close()
            else:
                if data:
                    # Echo the message and wait for more if still alive
                    connection.sendall(data)
                else:
                    # Data is empty and there will be no future messages
                    break

        # Shutdown connection properly
        connection.close()
