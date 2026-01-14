import serial
import re

SERIAL_PORT = "/dev/ttyUSB0"

def hex2int(hex: str) -> int:
    res = 0
    for c in hex:
        res = (res << 4) | ({"0": 0, "1": 1, "2": 2, "3": 3, "4":4, "5": 5, "6": 6, "7": 7, "8": 8, "9": 9, "A": 10, "B": 11, "C": 12, "D":13, "E":14, "F":15}[c])
    return res

s = serial.Serial()
buffer = []
while True:
    c = s.readline()
    if re.match("[0-9A-F]", c):
        buffer.append(hex2int(c))
    elif len(buffer):
        print(buffer)
        buffer.clear()