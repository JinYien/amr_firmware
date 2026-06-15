import ctypes
import numpy as np
import serial
import struct
from serial.tools import list_ports

NUM_OF_VARIABLES = 12
CHUNK_SIZE = 1
STATE_STRUCT = struct.Struct("<cc" + "d" * NUM_OF_VARIABLES * CHUNK_SIZE + "c")
CMD_INT_DATA_STRUCT = struct.Struct("<q")
CMD_DOUBLE_DATA_STRUCT = struct.Struct("<d")


def get_port(hw_ser):
    for port, desc, hwid in list_ports.comports():
        if hw_ser in hwid:
            return port
    return None


def read_packet(ser):
    raw_bytes = ser.read(STATE_STRUCT.size)
    if not raw_bytes:
        return 4, None

    start, end, identifier = raw_bytes[:1], raw_bytes[-1:], raw_bytes[1:2]
    if not (start == b"@" and end == b"\n"):
        _ = ser.readline()
        return 2, raw_bytes

    if identifier == b"s":
        data_array = np.frombuffer(raw_bytes[2:-1], dtype=ctypes.c_double).reshape((CHUNK_SIZE, NUM_OF_VARIABLES))
        return 0, data_array
    elif identifier == b"m":
        msg = raw_bytes[2:-1].decode().rstrip("\x00")
        return 1, msg

    return 3, raw_bytes


def write_packet_int(ser, command_type, command, value=0):
    packet = command_type + command + CMD_INT_DATA_STRUCT.pack(value) + b"\n"
    ser.write(packet)
    return packet


def write_packet_float(ser, command_type, command, value=0.0):
    packet = command_type + command + CMD_DOUBLE_DATA_STRUCT.pack(value) + b"\n"
    ser.write(packet)
    return packet


if __name__ == "__main__":
    port = get_port("VID:PID=16C0:0483")
    my_ser = serial.Serial(port, baudrate=115200, timeout=1)
    print("Testing serial communication...")

    print("\n Starting all systems...")
    send_packet = write_packet_int(my_ser, b"@", b"\x41")
    print(f"Sent: {send_packet}")
    print(f"Response: {read_packet(my_ser)}")

    print("\n Configuring left motor ...")
    send_packet = write_packet_int(my_ser, b"<", b"\x11", 1)
    print(f"Response: {read_packet(my_ser)}")
    send_packet = write_packet_float(my_ser, b"<", b"\x41", 100)
    print(f"Response: {read_packet(my_ser)}")

    print("\n Configuring right motor ...")
    send_packet = write_packet_int(my_ser, b">", b"\x11", 1)
    print(f"Response: {read_packet(my_ser)}")
    send_packet = write_packet_float(my_ser, b">", b"\x41", 50)
    print(f"Response: {read_packet(my_ser)}")

    print("\n Configuring middle motor ...")
    send_packet = write_packet_int(my_ser, b"*", b"\x13")
    print(f"Response: {read_packet(my_ser)}")
    send_packet = write_packet_int(my_ser, b"*", b"\x14", 1)
    print(f"Response: {read_packet(my_ser)}")
    send_packet = write_packet_float(my_ser, b"*", b"\x22", 0.5)
    print(f"Response: {read_packet(my_ser)}")

    print("\n Enabling data stream...")
    send_packet = write_packet_int(my_ser, b"@", b"\x21", 1)

    print("\n Reading sensor data (10 samples)...")
    for i in range(10):
        msg_type, data = read_packet(my_ser)
        if msg_type == 0:
            print(f" Sample {i+1}: {data[-1]}")

    my_ser.close()
    print("\n Test complete!")
