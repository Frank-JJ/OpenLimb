import serial
from pynput import keyboard
from timeit import default_timer as timer

# Configure your COM port and baud rate
COM_PORT = 'COM3'  # Change this to your COM port
BAUD_RATE = 115200   # Change this to match your UART device configuration

# Open the serial connection wtih com pot and baud rate, as well as 8 data bits, no parity, one stop bit
ser = serial.Serial(COM_PORT, BAUD_RATE, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

# Control keys (up, down, up, down, up, down)
# These are used for moving servo "up" or "down"
# 'q' and 'a' are for moving servo 1 "up" and "down", and so on...
control_keys = [keyboard.KeyCode.from_char(c) for c in 'qawsed']

# Sending an array as a byte array makes it easy to use the data directly as a simple array on the arduino
def send_as_bytearray(arr):
    ser.write(bytearray(arr))

# Initialize position array to zero
NUM_SERVOS = 3
pos = [0 for i in range(NUM_SERVOS)]
send_as_bytearray(pos)

# Multiplier to comand constant rotation speed of servo
# 1 is slow
vel_mult = 5

def on_press(key):
    # Initialize change to zero
    change = [0 for i in range(NUM_SERVOS)]
    
    # Check key type
    try:
        key_pressed = control_keys.index(key)
    except:
        # Key not in control_keys
        return
    
    # Set change in position array
    change[int(key_pressed / 2)] = 1
    if key_pressed % 2:
        change[int(key_pressed / 2)] *= -1
        
    global pos
    # Update position array and send it
    for index in range(NUM_SERVOS):
        if pos[index] + change[index]*vel_mult <= 180 and pos[index] + change[index]*vel_mult >= 0:
            pos[index] += change[index]*vel_mult
    send_as_bytearray(pos)
    

def on_release(key):
    return
    # print("release")

listener = keyboard.Listener(on_press=on_press, on_release=on_release)

delta_time = 0
run_time = 0

def gaitY_1():
    global delta_time, run_time
    delta_time = timer() - delta_time
    run_time += delta_time
    # movements = {}
    # # Move left leg
    # if run_time < 1:
    #     pos[0] += 


try:
    # Start listener that allows getting keyboard input
    ser.reset_input_buffer()
    listener.start()
    
    while True:
        # Chech and read messages from serial connection
        if ser.in_waiting > 0:
            c = ser.readline()
            try:
                print(c.decode())
            except:
                print(c)

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()