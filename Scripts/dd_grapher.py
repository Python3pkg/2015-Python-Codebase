import socket
import pickle
import matplotlib.pyplot as plt

def main():
    s = socket.socket()
    s.connect(("roborio-4819.local", 2222))
    str_data = b""
    while True:
        packet = s.recv(1024)
        if packet.endswith(b'FIN'):
            str_data = str_data + packet[:-3]
            break
        str_data = str_data + packet
    data = pickle.loads(str_data)
    speed_data = data["data"]
    sample_del = 1/data["rate"]
    x_points = []
    y1_points = []
    y2_points = []
    y3_points = []
    ct = 0
    for y1, y2 in speed_data:
        y1_points.append(y1)
        y2_points.append(y2)
        y3_points.append(abs(y1 - y2))
        x_points.append(ct)
        ct += sample_del
    plt.plot(x_points, y1_points, "b", label="control_y_speed")
    plt.plot(x_points, y2_points, "r", label="sensor_y_speed")
    plt.plot(x_points, y3_points, "g", label="delta_magnitude")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.show()




if __name__ == "__main__":
    main()