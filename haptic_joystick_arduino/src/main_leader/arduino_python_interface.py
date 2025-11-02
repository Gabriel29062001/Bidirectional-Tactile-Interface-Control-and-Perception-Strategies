import serial

# Configure the serial port
port = '/dev/ttyUSB0'  # Change this to the appropriate port
baudrate = 115200

# Create a serial object
ser = serial.Serial(port, baudrate)

# Wait for the Arduino to reset
ser.setDTR(False)
ser.open()
ser.setDTR(True)

# Send data to the Arduino
data_to_send = [1.23, 4.56, 7.89]  # Example float values
data_string = ' '.join(str(value) for value in data_to_send)
ser.write(data_string.encode())

# Read data from the Arduino
data_received = ser.readline().decode().strip()
print('Data received:', data_received)

# Close the serial connection
ser.close()
