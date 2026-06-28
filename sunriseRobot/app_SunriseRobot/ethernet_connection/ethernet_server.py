import time
import queue
import socket
import threading

import args
import utils
import global_constants as gc
from robot_link import protocol
from function_calls.function_caller import FunctionCaller


class ConnectionHandler:
    def __init__(self, function_caller: FunctionCaller, connection, address, number: int = None, verbose: int = 0):
        self.function_caller = function_caller
        self.connection = connection
        self.address = address
        self.number = number
        self.verbose = verbose
        # commands to send to this client (RDK X3 -> Jetson direction), drained by sender()
        self.outgoing = queue.Queue()

    def enqueue_command(self, name: str, args=None) -> None:
        """Queue a command to be sent to this client (RDK X3 -> Jetson direction)."""
        self.outgoing.put((name, args))

    def receiver(self):
        """Receive JSON commands from the client and dispatch them via the FunctionCaller."""
        while self.connection:
            command = protocol.recv_command(self.connection)
            if command is None:
                # the client closed the connection (or framed an empty message)
                self.close()
                break
            try:
                self.function_caller.call_function(function_name=command['name'], kwargs=command['args'])
            except Exception as e:
                utils.print_exception(exception=e, message='Error when executing received function')

    def sender(self):
        """Send queued commands to the client (RDK X3 -> Jetson direction)."""
        while self.connection:
            try:
                name, command_args = self.outgoing.get(timeout=0.3)
            except queue.Empty:
                continue
            try:
                protocol.send_command(self.connection, name, command_args)
            except Exception as e:
                utils.print_exception(exception=e, message='Ethernet server sender error')
                self.close()
                break

    def start(self):
        # Start the receiver and sender threads (the channel is full-duplex on one socket)
        if self.verbose >= 2:
            print('Starting client handler...')
        suffix = f'_{self.number}' if self.number is not None else ''
        receiver_thread = threading.Thread(
            target=self.receiver, name=f'ethernet_server_receiver{suffix}', daemon=True)
        sender_thread = threading.Thread(
            target=self.sender, name=f'ethernet_server_sender{suffix}', daemon=True)
        receiver_thread.start()
        sender_thread.start()
        if self.verbose >= 1:
            print(f'Receiver thread started: "{receiver_thread.name}"')
            print(f'Sender thread started: "{sender_thread.name}"')

    def close(self) -> None:
        if self.connection:
            self.connection.close()
            self.connection = None
            if self.verbose >= 1:
                print("Connection closed.")


class EthernetServer:
    def __init__(self, robot_head, robot_body, arm, light, **kwargs):
        """
        Initializes the Ethernet server with the specified host and port.
        :param host: The hostname or IP address of the server.
        :param port: The port number on which the server is listening.
        """
        self.robot_head = robot_head
        self.robot_body = robot_body
        self.arm = arm
        self.light = light
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'ethernet_server.yaml', **kwargs)
        self.host = parameters['host']
        self.port = parameters['port']
        self.retry_interval = parameters['retry_interval']
        self.socket = None
        self.active_connections = []
        self.connection_counter = 0
        self.is_active = False
        self.verbose = parameters['verbose']
        self.function_caller = FunctionCaller(
            robot_head=robot_head,
            robot_body=robot_body,
            arm=arm,
            light=light,
            verbose=self.verbose,
        )

    def stop(self) -> None:
        """
        Stops the Ethernet server and closes all active connections.
        """
        self.is_active = False
        if self.verbose >= 2:
            print("Stopping the server...")
        for connection in self.active_connections:
            connection.close()
        self.active_connections = []
        if self.socket:
            self.socket.close()
            self.socket = None
        if self.verbose >= 1:
            print("Server stopped. All connections closed.")

    def send_command(self, name: str, args=None) -> None:
        """
        Send a command to the connected client(s) (RDK X3 -> Jetson direction), e.g. to actuate the
        Jetson-side headlight. Prunes any connections that have since closed.
        """
        for handler in list(self.active_connections):
            if handler.connection is None:
                self.active_connections.remove(handler)
            else:
                handler.enqueue_command(name, args)

    def start(self):
        """
        Starts the Ethernet server, listening for incoming connections.
        Accepts new connections and starts a handler for each connection.
        """
        server_thread = threading.Thread(target=self.wait_connections, name='ethernet_server')
        server_thread.start()

    def listen(self) -> None:
        listening = False
        if self.verbose >= 2:
            print(f'Starting server on {self.host}:{self.port}')
        while not listening:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # prevent "Address already in use" error
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.bind((self.host, self.port))
                self.socket.listen()
                if self.verbose >= 2:
                    print(f"\tServer listening on {self.host}:{self.port}")
                self.is_active = True
                listening = True
            except socket.error as e:
                utils.print_exception(exception=e, message='Error starting server')
                if self.verbose >= 1:
                    print(f'\tServer failed starting. Retrying in {self.retry_interval} seconds...')
                listening = False
            time.sleep(self.retry_interval)

    def wait_connections(self):
        self.listen()

        while self.is_active:
            connection, address = self.socket.accept()
            self.connection_counter += 1
            new_connection = ConnectionHandler(
                function_caller=self.function_caller,
                connection=connection,
                address=address,
                number=self.connection_counter,
                verbose=self.verbose
            )
            self.active_connections.append(new_connection)
            new_connection.start()

        if self.socket:
            self.socket.close()
        if self.verbose >= 1:
            print("Connection closed.")
