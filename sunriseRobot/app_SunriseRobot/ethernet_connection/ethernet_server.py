import json
import time
import socket
import threading

import args
import utils
import global_constants as gc
from function_calls.function_caller import FunctionCaller


class ConnectionHandler:
    def __init__(self, robot_head, robot_body, arm, light, connection, address, number: int = None, verbose: int = 0):
        self.connection = connection
        self.address = address
        self.number = number
        self.function_caller = FunctionCaller(
            robot_head=robot_head,
            robot_body=robot_body,
            arm=arm,
            light=light,
            verbose=verbose,
        )

    def receive_data(self) -> str | None:
        try:
            data = self.connection.recv(1024)
            if not data:
                print("No data received.")
                return None
            print(f"server received: {data.decode()}")
            return data.decode()
        except Exception as e:
            utils.print_exception(exception=e, message='Ethernet server "receive_data" error')

    def receive_function_call(self) -> dict | None:
        try:
            # First, receive the 4-byte length prefix
            length_prefix = self.connection.recv(4)
            if not length_prefix:
                print("Client disconnected unexpectedly during length prefix reception.")
                return None

            message_length = int.from_bytes(bytes=length_prefix, byteorder='big')

            # Receive the actual JSON data based on the length
            received_bytes = b''
            while len(received_bytes) < message_length:
                packet = self.connection.recv(message_length - len(received_bytes))
                if not packet:
                    print("Client disconnected unexpectedly during data reception.")
                    return None
                received_bytes += packet

            if not received_bytes:  # Handle cases where packet was empty
                print("No data received after length prefix.")
                return None

            # Decode the bytes back to a JSON string
            json_string = received_bytes.decode('utf-8')

            # Deserialize the JSON string back into a Python dictionary
            received_data = json.loads(json_string)

            print(f"Server received: {received_data}")

            # Now you have the function call data as a dictionary:
            function_name = received_data.get("name")
            function_args = received_data.get("args")

            print(f'\tFunction Name: {function_name}')
            print(f'\tFunction Args: {function_args}')

            # Send an acknowledgment back to the client if needed
            # self.connection.sendall(b'ACK received function call')
        except Exception as e:
            utils.print_exception(exception=e, message='Ethernet server "receive_data" error')

    def send_data(self, data) -> None:
        try:
            self.connection.sendall(data.encode())
            print(f"server sent: {data}")
        except Exception as e:
            utils.print_exception(exception=e, message='Ethernet server "send_data" error')

    def receiver(self) :
        while self.connection:
            decoded_data = self.receive_function_call()
            if decoded_data is not None:
                try:
                    print(f'Executing function "{decoded_data.name}" with parameters {decoded_data.args}')
                    self.function_caller.call_function(function_name=decoded_data.name, kwargs=decoded_data.args)
                except Exception as e:
                    utils.print_exception(exception=e, message='Error when executing received function')
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
                self.send_data(message_to_send)
                time.sleep(0.01)

    def start(self):
        # Start the receiver and sender threads
        print('Starting Client handler...')
        if self.number is not None:
            receiver_thread = threading.Thread(target=self.receiver, name=f'ethernet_client_receiver_{self.number}')
            # sender_thread = threading.Thread(target=self.sender, name=f'ethernet_client_sender_{self.number}')
        else:
            receiver_thread = threading.Thread(target=self.receiver)
            # sender_thread = threading.Thread(target=self.sender)

        receiver_thread.start()
        print(f'Receiver thread started: "{receiver_thread.name}"')
        # sender_thread.start()
        # print(f'Sender thread started: "{sender_thread.name}"')

        # Keep the main thread alive or join the other threads
        # receiver_thread.join()
        # sender_thread.join()

        # self.close()

    def close(self) -> None:
        if self.connection:
            self.connection.close()
            self.connection = None
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
        self.socket = None
        self.active_connections = []
        self.connection_counter = 0
        self.is_active = False
        self.verbose = parameters['verbose']

    def stop(self) -> None:
        """
        Stops the Ethernet server and closes all active connections.
        """
        self.is_active = False
        print("Stopping the server...")
        for connection in self.active_connections:
            connection.close()
        self.active_connections = []
        print("All connections closed.")

    def start(self):
        """
        Starts the Ethernet server, listening for incoming connections.
        Accepts new connections and starts a handler for each connection.
        """
        self.is_active = True
        server_thread = threading.Thread(target=self.wait_connections, name='ethernet_server')
        server_thread.start()

    def wait_connections(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f'Starting server on {self.host}:{self.port}')
        self.socket.bind((self.host, self.port))
        self.socket.listen()
        print(f"Server listening on {self.host}:{self.port}")

        while self.is_active:
            connection, address = self.socket.accept()
            self.connection_counter += 1
            new_connection = ConnectionHandler(
                robot_head=self.robot_head,
                robot_body=self.robot_body,
                arm=self.arm,
                light=self.light,
                connection=connection,
                address=address,
                number=self.connection_counter,
                verbose=self.verbose
            )
            self.active_connections.append(new_connection)
            new_connection.start()

        if self.socket:
            self.socket.close()
            print("Connection closed.")


if __name__ == "__main__":
    ethernet_server = EthernetServer()
    ethernet_server.start()
