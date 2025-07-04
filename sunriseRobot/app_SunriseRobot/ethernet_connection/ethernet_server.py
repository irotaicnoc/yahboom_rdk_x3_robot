import json
import time
import socket
import threading

import args
import utils
import global_constants as gc
from function_calls.function_caller import FunctionCaller


class ConnectionHandler:
    def __init__(self, function_caller: FunctionCaller, connection, address, number: int = None, verbose: int = 0):
        self.function_caller = function_caller
        self.connection = connection
        self.address = address
        self.number = number
        self.verbose = verbose

    def receive_data(self):
        try:
            data = self.connection.recv(1024)
            if not data:
                if self.verbose >= 1:
                    print("No data received.")
                return None
            if self.verbose >= 3:
                print(f"server received: {data.decode()}")
            return data.decode()
        except Exception as e:
            utils.print_exception(exception=e, message='Ethernet server "receive_data" error')
            self.close()

    def receive_function_call(self) -> dict:
        try:
            # First, receive the 4-byte length prefix
            length_prefix = self.connection.recv(4)
            if not length_prefix:
                if self.verbose >= 1:
                    print("Client disconnected unexpectedly during length prefix reception.")
                self.close()

            message_length = int.from_bytes(bytes=length_prefix, byteorder='big')

            # Receive the actual JSON data based on the length
            received_bytes = b''
            while len(received_bytes) < message_length:
                packet = self.connection.recv(message_length - len(received_bytes))
                if not packet:
                    if self.verbose >= 1:
                        print("Client disconnected unexpectedly during data reception.")
                    self.close()
                received_bytes += packet

            if not received_bytes:  # Handle cases where packet was empty
                if self.verbose >= 1:
                    print("No data received after length prefix.")
                self.close()

            # Decode the bytes back to a JSON string
            json_string = received_bytes.decode('utf-8')

            # Deserialize the JSON string back into a Python dictionary
            received_data = json.loads(json_string)

            # Now you have the function call data as a dictionary:
            function_call = {
                "name": received_data.get("name"),
                "args": received_data.get("args"),
            }
            # utils.pretty_print_dict(function_call)

            # Send an acknowledgment back to the client if needed
            # self.connection.sendall(b'ACK received function call')
            return function_call

        except Exception as e:
            utils.print_exception(exception=e, message='Ethernet server "receive_data" error')
            self.close()

    def receiver(self) :
        while self.connection:
            decoded_data = self.receive_function_call()
            if decoded_data is not None:
                try:
                    self.function_caller.call_function(function_name=decoded_data['name'], kwargs=decoded_data['args'])
                except Exception as e:
                    utils.print_exception(exception=e, message='Error when executing received function')
                    self.close()
            else:
                time.sleep(0.3)

    def sender(self):
        while self.connection:
            # TODO: send messages
            message_to_send = None
            if message_to_send is None:
                time.sleep(0.3)
                continue
            else:
                self.connection.sendall(message_to_send.encode())
                time.sleep(0.01)

    def start(self):
        # Start the receiver and sender threads
        if self.verbose >= 2:
            print('Starting Client handler...')
        if self.number is not None:
            receiver_thread = threading.Thread(target=self.receiver, name=f'ethernet_client_receiver_{self.number}')
            # sender_thread = threading.Thread(target=self.sender, name=f'ethernet_client_sender_{self.number}')
        else:
            receiver_thread = threading.Thread(target=self.receiver)
            # sender_thread = threading.Thread(target=self.sender)

        receiver_thread.start()
        if self.verbose >= 1:
            print(f'Receiver thread started: "{receiver_thread.name}"')
        # sender_thread.start()
        # if self.verbose >= 1:
        #     print(f'Sender thread started: "{sender_thread.name}"')

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
