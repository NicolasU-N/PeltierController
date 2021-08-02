import Serial
import time

serialcomm = serial.Serial('COM4', 115200);
serialcomm.timeout = 1

while True:
  i = input("->input command: ").strip()
  serialcomm.write(i.encode())
  time.sleep(0.5)
  print(serialcomm.readline().decode('ascii'))

serialcomm.close()