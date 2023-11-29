import serial
import time
import numpy as np
import matplotlib.pyplot as plt

"""
Standard deviation of 'acceleromter' values: 0.18854484877609357
Standard deviation of 'gyro x' values: 0.09413373465447974
"""

if __name__ == "__main__":
    # Open serial port
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    ay_list = []
    gx_list = []

    num_of_samples = 200
    # Read and print lines from the serial port in an infinite loop
    for i in range(num_of_samples):
        line = ser.readline()
        print(line.decode('utf-8').strip())
        # Split the line into fields
        fields = line.decode('utf-8').strip().split('|')
        # Filter out the 'ay' and 'gx' fields
        acc_deg_field = [field.split(': ')[1] for field in fields if 'acc_deg' in field][0]
        gx_field = [field.split(': ')[1] for field in fields if 'gyro_deg' in field][0]
        print("acc_deg_field:", acc_deg_field)
        print("gx_field:", gx_field)
        # Append them into separate lists
        ay_list.append(float(acc_deg_field))
        gx_list.append(float(gx_field))

        # time.sleep(0.01)  # Delay for 10 milliseconds

    ser.close()

    # Calculate the standard deviation for both lists
    ay_std_dev = np.std(ay_list)
    gx_std_dev = np.std(gx_list)

    # Print the standard deviations
    print(f"Standard deviation of 'acceleromter' values: {ay_std_dev}")
    print(f"Standard deviation of 'gyro x' values: {gx_std_dev}")

    x = np.linspace(0, 0.1, num_of_samples)

    print(f"Size of x: {len(x)}")
    print(f"Size of ay_list: {len(ay_list)}")
    print(f"Size of gx_list: {len(gx_list)}")

    plt.plot(x, ay_list, label='accelerometer values')
    plt.plot(x, gx_list, label='kalmans filter values')
    plt.xlabel('Time (0.1s intervals)')
    plt.ylabel('Values')
    plt.title('Accelerometer and Kalman Filter Values over Time')
    plt.legend()
    plt.show()




