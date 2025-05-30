import socket
import struct
from collections import deque

HOST, PORT = '10.0.2.2', 10001
SYNC = b'\xAA\x55'
SIZE = 2 + 7   # sync + payload
WINDOW_SIZE = 1  # Change this for a different moving average window

def recv_packet(sock):
    # find SYNC in stream
    buf = b''
    while True:
        b = sock.recv(1)
        if not b:
            raise ConnectionError
        buf += b
        if buf.endswith(SYNC):
            break
    # now read the 7-byte payload
    payload = sock.recv(7)
    while len(payload) < 7:
        part = sock.recv(7 - len(payload))
        if not part:
            raise ConnectionError
        payload += part
    return payload

# For storing last N adc values
window = deque(maxlen=WINDOW_SIZE)

with socket.create_connection((HOST, PORT)) as s:
    with open("loadcelldata.csv", 'w') as file:
        file.write("ts,avg\n")

        while True:
            data = recv_packet(s)
            ts, b0, b1, b2 = struct.unpack('<I3B', data)
            raw = (b0 << 16) | (b1 << 8) | b2
            if raw & (1 << 23):
                raw -= (1 << 24)

            window.append(raw)
            avg = sum(window) / len(window)
            print(f"t={ts}, moving_avg={avg/-15:.2f}")
            file.write(str(ts) + "," + str(avg/-15) + "\n")
