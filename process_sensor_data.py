import pandas as pd
import matplotlib.pyplot as plt
plt.show()

# Load in the data
data_gps = pd.read_csv("config/log/Graph1.txt") 
data_imu = pd.read_csv("config/log/Graph2.txt") 

# Take a look at the data
fig, axes = plt.subplots(nrows=1, ncols=2)
data_gps.plot(ax=axes[0], x="time", y=" Quad.GPS.X", kind = 'line')
data_imu.plot(ax=axes[1], x="time", y=" Quad.IMU.AX", kind = 'line')

# Calculate standard deviation
sigma_gps = data_gps.std(axis=0)[" Quad.GPS.X"]
print("GPS X standard deviation = {:.3}".format(sigma_gps))
sigma_imu = data_imu.std(axis=0)[" Quad.IMU.AX"]
print("IMU standard deviation = {:.3}".format(sigma_imu))

axes[0].plot([0, data_gps["time"].iloc[-1]], [sigma_gps, sigma_gps])
axes[0].plot([0, data_gps["time"].iloc[-1]], [-sigma_gps, -sigma_gps])

axes[1].plot([0, data_imu["time"].iloc[-1]], [sigma_imu, sigma_imu])
axes[1].plot([0, data_imu["time"].iloc[-1]], [-sigma_imu, -sigma_imu])


# Check some other sensors
sigma_gps_vx = data_gps.std(axis=0)[" Quad.GPS.vx"]
print("GPS VX standard deviation = {:.3}".format(sigma_gps_vx))

sigma_gps_z = data_gps.std(axis=0)[" Quad.GPS.z"]
print("GPS Z standard deviation = {:.3}".format(sigma_gps_z))

sigma_gps_vz = data_gps.std(axis=0)[" Quad.GPS.vz"]
print("GPS VZ standard deviation = {:.3}".format(sigma_gps_vz))


plt.show()