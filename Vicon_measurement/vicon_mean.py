import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Charger le fichier CSV dans un DataFrame pandas
df = pd.read_csv('/home/gabriel/ros2_ws/src/haptic_joystick/Vicon_measurement/240523_gabriel_exp_4.csv')

# Afficher les noms des colonnes
print(df.columns)




translation_z = df["TZ"]
translation_z = translation_z -translation_z[0]
translation_z_2 = df["TZ.1"]
translation_z_2 = translation_z_2 - translation_z_2[0]
translation_z_2[translation_z_2>max(translation_z)] = max(translation_z)
translation_z_2[translation_z_2<min(translation_z)] = min(translation_z)
frame = df["Frame"]

translation_rx = df["RX"]
translation_rx = translation_rx -translation_rx[0]
translation_rx_2 = df["RX.1"]
translation_rx_2 = translation_rx_2 - translation_rx_2[0]
translation_rx_2[translation_rx_2>max(translation_rx)] = max(translation_rx)
translation_rx_2[translation_rx_2<min(translation_rx)] = min(translation_rx)


translation_ry = df["RY"]
translation_ry = translation_ry -translation_ry[0]
translation_ry_2 = df["RY.1"]
translation_ry_2 = translation_ry_2 - translation_ry_2[0]
translation_ry_2[translation_ry_2>max(translation_ry)] = max(translation_ry)
translation_ry_2[translation_ry_2<min(translation_ry)] = min(translation_ry)


translation_z_mean = np.mean(translation_z)
translation_z_2_mean = np.mean(translation_z_2)

print(translation_z_mean)
print(translation_z_2_mean)
error_z = abs(translation_z_2_mean-translation_z_mean)/translation_z_mean*100
print(error_z)


translation_rx_mean = np.mean(translation_rx)
translation_rx_2_mean = np.mean(translation_rx_2)


print(translation_rx_mean)
print(translation_rx_2_mean)
error_rx = abs(translation_rx_2_mean-translation_rx_mean)/translation_rx_mean*100
print(np.abs(error_rx))

translation_ry_mean = np.mean(translation_ry)
translation_ry_2_mean = np.mean(translation_ry_2)

print(translation_ry_mean)
print(translation_ry_2_mean)
error_ry = abs(translation_ry_2_mean-translation_ry_mean)/translation_ry_mean*100
print(np.abs(error_ry))


error_z2 = np.mean(np.abs(translation_z)- np.abs(translation_z_2))/np.mean(translation_z)