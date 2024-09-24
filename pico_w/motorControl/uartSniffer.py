import serial

# Configure your COM port and baud rate
COM_PORT = 'COM3'  # Change this to your COM port
BAUD_RATE = 115200   # Change this to match your UART device configuration

# Open the serial connection wtih com pot and baud rate, as well as 8 data bits, no parity, one stop bit
ser = serial.Serial(COM_PORT, BAUD_RATE, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

try:
    while True:
        data = ser.read() #Read one byte at a time
        print(data.hex())
except:
    ser.close()