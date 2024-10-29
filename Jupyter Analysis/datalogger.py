from datetime import datetime
import serial

baud = 1000000
port = 'COM3'
file = "data.csv"
header = "Date, Time,Atmospheric pressure,Temperature\n"

'''
file = "comparisonTwoSensors.csv"
header = "Date,Time,Atmospheric pressure 1,Temperature 1,Atmospheric pressure 2,Temperature 2\n"
'''

ser = serial.Serial(port, baud)
ser.flushInput()
file = open(file, "a")
file.write(header)

while True:

    current_date_time = datetime.now()
    formatted_time = current_date_time.strftime("%H:%M, ")
    formatted_date = current_date_time.strftime("%Y/%m/%d, ")
    data = formatted_date + formatted_time + str(ser.readline().decode("utf-8"))
    file = open(file, "a")
    file.write(data)
    