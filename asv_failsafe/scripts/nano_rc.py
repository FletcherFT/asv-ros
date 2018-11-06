import serial
import struct

def main():
    try:
        fmt = ">BBBBBB"
        size = struct.calcsize(fmt)
        with serial.Serial("/dev/ttyUSB0",9600,timeout=1) as ser:
            while True:
                line = ser.readline().rstrip()
                if not line is None and len(line)==size:
                    data = struct.unpack(fmt,line)
                    print(data)
    except KeyboardInterrupt:
        pass


if __name__=="__main__":
    main()