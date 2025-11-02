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
frame = frame*1/300


translation_z_mean = np.mean(translation_z)
translation_z_2_mean = np.mean(translation_z_2)

print(translation_z_mean)
print(translation_z_2_mean)
error_z = abs(translation_z_2_mean-translation_z_mean)/translation_z_mean*100
print(error_z)



if pd.api.types.is_numeric_dtype(translation_z) and pd.api.types.is_numeric_dtype(frame):

    plt.plot(frame, translation_z,label = "Follower (Interface tactile 2)", color = "blue")
    plt.plot(frame, translation_z_2,label = "Leader (Interface tactile 1)", color = "orange")
    # Ajouter un titre et des labels
    plt.legend()
    plt.title('Translation Z vs Frame')
    plt.xlabel('Time (s)')
    plt.ylabel('Translation Z')
    plt.show(block=True)

indice_max_z = np.where(translation_z >= 15)

print(indice_max_z)
print("test")

interval_z = []
interval_z.append(indice_max_z[0][0])
print(interval_z)

for i in range(len(indice_max_z[0])-1):
    if indice_max_z[0][i+1] - indice_max_z[0][i] > 1:
            interval_z.append(indice_max_z[0][i+1])
            
#delay = (indice_max_z[0][0]-indice_max_z_2[0][0])*1/300
#print(delay)

print(interval_z)

for k in range(len(interval_z)-1):
    plt.figure(k)
    print(interval_z[k], interval_z[k+1])
    frame_plot = frame[interval_z[k]:interval_z[k+1]]
    translation_z_plot = translation_z[interval_z[k]:interval_z[k+1]]
    translation_z_2_plot = translation_z_2[interval_z[k]:interval_z[k+1]]

    indice_max_z_plot = np.where(translation_z_plot == max(translation_z_plot))
    indice_max_z_2_plot = np.where(translation_z_2_plot == max(translation_z_2_plot))
    delay = abs((indice_max_z_plot[0][0]-indice_max_z_2_plot[0][0])*1/300)
    print(indice_max_z_plot[0][0])
    print(indice_max_z_2_plot[0])
    print(delay)


    plt.plot(frame_plot, translation_z_plot,label = "Follower (Interface tactile 2)", color = "blue")
    plt.plot(frame_plot, translation_z_2_plot,label = "Leader (Interface tactile 1)", color = "orange")
    # Ajouter un titre et des labels
    plt.legend()
    plt.title('Translation Z vs Frame')
    plt.xlabel('Time (s)')
    plt.ylabel('Translation Z')
    plt.show(block=True)

0.15333333333333332
0.023333333333333334
0.016666666666666666
0.38
0.63
0.043333333333333335
0.016666666666666666