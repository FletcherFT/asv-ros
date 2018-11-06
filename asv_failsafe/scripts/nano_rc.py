import serial
import struct

def main():
    try:
        fmt = "cHHHHHH"
        with serial.Serial("/dev/ttyACM0",9600,timeout=1) as ser:
            while True:
                line = ser.readline()
                if line not None:
                    data = struct.unpack(fmt,line)
                    print(data)

if __name__=="__main__":
    main()