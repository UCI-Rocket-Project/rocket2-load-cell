import socket
import struct
import time

# Define Arduino server settings
ARDUINO_IP = '10.0.2.2'  # IP address of the Arduino
PORT = 10001                  # Port to connect to

# Define the packet format
PACKET_FORMAT = '<Lf'  # Add 4 padding bytes (or use '12s' for raw data)

def connect_to_arduino():
    try:
        # Create a socket connection
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((ARDUINO_IP, PORT))
        print(f"Connected to Arduino at {ARDUINO_IP}:{PORT}")
        return client_socket
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def receive_data(client_socket):
    try:
        # The size of the packet in bytes (unsigned long + float)
        packet_size = struct.calcsize(PACKET_FORMAT)

        while True:
            # Receive the data
            data = client_socket.recv(packet_size)
            if len(data) == packet_size:
                # Unpack the data according to the specified format
                timestamp, force = struct.unpack(PACKET_FORMAT, data)
                print(f"Timestamp: {timestamp}, Force: {force}")
            else:
                print(packet_size)
                print(len(data))
                print("Incomplete data received")
                
    except Exception as e:
        print(f"Error receiving data: {e}")
    finally:
        client_socket.close()
        print("Connection closed.")

if __name__ == "__main__":
    socket_connection = connect_to_arduino()
    if socket_connection:
        receive_data(socket_connection)
