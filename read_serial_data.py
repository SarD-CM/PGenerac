import serial
from datetime import datetime
from multiprocessing import Process, Value

def read_serial_data(data_list):
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port and baud rate as per your setup
    while True:
        if ser.in_waiting:
            data = ser.readline().decode().rstrip()
            current_time = datetime.now()
            data_list.append([current_time] + data.split(','))

if __name__ == '__main__':
    data_list = Value('i', [])  # Shared variable to hold the list of lists
    process1 = Process(target=read_serial_data, args=(data_list,))
    process1.start()
    process1.join()

