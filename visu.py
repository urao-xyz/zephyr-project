import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyUSB0', 115200)
data = []

for i in range(50):
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue
    try:
        r, ir = map(int, line.split())
        data.append(r)
    except ValueError:
        continue

plt.plot(data)
plt.title("Signal PPG")
plt.xlabel("Échantillons")
plt.ylabel("Amplitude")
plt.savefig("ppg_signal.png")
print("Graphe sauvegardé sous ppg_signal.png")