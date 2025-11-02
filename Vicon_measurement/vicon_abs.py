import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Charger le fichier CSV dans un DataFrame pandas
df = pd.read_csv("/home/gabriel/ros2_ws/src/haptic_joystick/Vicon_measurement/240523_gabriel_exp_4.csv")

# Afficher les noms des colonnes
print(df.columns)

# Assurez-vous que les colonnes existent et contiennent des données numériques
if 'TZ' in df.columns and 'Frame' in df.columns:
    translation_z = df["TZ"]
    translation_z = translation_z -translation_z[0]
    translation_z = translation_z[10000:15000]
    translation_z_2 = df["TZ.1"]
    translation_z_2 = translation_z_2 - translation_z_2[0]
    translation_z_2 = translation_z_2[10000:15000]
    translation_z_2[translation_z_2>max(translation_z)] = max(translation_z)
    translation_z_2[translation_z_2<min(translation_z)] = min(translation_z)
    frame = df["Frame"]
    frame = (frame[10000:15000]-10000)*1/300




    translation_rx = df["RX"]
    translation_rx = translation_rx -translation_rx[0]
    translation_rx = translation_rx[10000:15000]

    translation_rx_2 = df["RX.1"]
    translation_rx_2 = translation_rx_2 - translation_rx_2[0]
    translation_rx_2 = translation_rx_2[10000:15000]



    translation_ry = df["RY"]
    translation_ry = translation_ry -translation_ry[0]
    translation_ry = translation_ry[10000:15000]

    translation_ry_2 = df["RY.1"]
    translation_ry_2 = translation_ry_2 - translation_ry_2[0]
    translation_ry_2 = translation_ry_2[10000:15000]


    
    # Vérifiez que les données sont numériques
    if pd.api.types.is_numeric_dtype(translation_z) and pd.api.types.is_numeric_dtype(frame):
        plt.plot(frame, translation_z, label = "Follower (Interface tactile 1)", color = "blue")
        plt.plot(frame, translation_z_2, label = "Leader (Interface tactile 2)", color = "orange")
        plt.legend()
        # Ajouter un titre et des labels
        plt.xlabel('Time (s)')
        plt.ylabel('Absolute Translation in z-direction (mm)')
        plt.show(block=True)

        plt.figure(2)
        plt.plot(frame, translation_rx, label = "Follower (Interface tactile 1)", color = "blue")
        plt.plot(frame, translation_rx_2,label = "Leader (Interface tactile 2)", color = "orange")
        plt.legend()
        # Ajouter un titre et des labels
        plt.xlabel('Time (s)')
        plt.ylabel('Rotation in x-direction (rad)')
        plt.show(block=True)
      

        plt.figure(3)
        plt.plot(frame, translation_ry, label = "Follower (Interface tactile 1)", color = "blue")
        plt.plot(frame, translation_ry_2, label = "Leader (Interface tactile 2)", color = "orange")
        plt.legend()
        # Ajouter un titre et des labels
        plt.xlabel('Time (s)')
        plt.ylabel('Rotation in y-direction (rad)')
        plt.show(block=True)

      
        
    else:
        print("Les colonnes 'TZ' et 'Frame' doivent contenir des données numériques.")
else:
    print("Les colonnes 'TZ' et/ou 'Frame' ne sont pas présentes dans le fichier CSV.")



translation_z_mean = np.mean(translation_z)
translation_z_2_mean = np.mean(translation_z_2)

print(translation_z_mean)
print(translation_z_2_mean)
error_z = abs(translation_z_2_mean-translation_z_mean)/translation_z_mean*100
print("Error",error_z)


translation_rx_mean = np.mean(translation_rx)
translation_rx_2_mean = np.mean(translation_rx_2)

print(translation_rx_mean)
print(translation_rx_2_mean)
error_rx = abs(translation_rx_2_mean-translation_rx_mean)/translation_rx_mean*100
print("Error",np.abs(error_rx))

translation_ry_mean = np.mean(translation_ry)
translation_ry_2_mean = np.mean(translation_ry_2)

print(translation_ry_mean)
print(translation_ry_2_mean)
error_ry = abs(translation_ry_2_mean-translation_ry_mean)/translation_ry_mean*100
print("Error",np.abs(error_ry))