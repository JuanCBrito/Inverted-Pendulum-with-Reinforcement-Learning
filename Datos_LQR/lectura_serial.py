import serial

ser = serial.Serial('COM6', 115200, timeout=1)
with open('Datos_LQR/LQR_Pendulo_2_1.csv', 'w') as f:
    while True:
        line = ser.readline().decode('ascii', errors='ignore')
        if line:
            f.write(line)
            print(line.strip())