import pandas as pd
import matplotlib.pyplot as plt

# Charger le fichier CSV dans un DataFrame pandas
df = pd.read_csv('/home/gabriel/ros2_ws/src/haptic_joystick/Vicon_measurement/240523_gabriel_exp_4.csv')

# Afficher les noms des colonnes
print(df.columns)



# Assurez-vous que les colonnes existent et contiennent des données numériques
if 'TZ' in df.columns and 'Frame' in df.columns:



    translation_z = df["TZ"]
    translation_z = translation_z -translation_z[0]
    translation_z_2 = df["TZ.1"]
    translation_z_2 = translation_z_2 - translation_z_2[0]
    frame = df["Frame"]

    translation_rx = df["RX"]
    translation_rx = translation_rx -translation_rx[0]
    translation_rx_2 = df["RX.1"]
    translation_rx_2 = translation_rx_2 - translation_rx_2[0]


    translation_ry = df["RY"]
    translation_ry = translation_ry -translation_ry[0]
    translation_ry_2 = df["RY.1"]
    translation_ry_2 = translation_ry_2 - translation_ry_2[0]


    
    # Vérifiez que les données sont numériques
    if pd.api.types.is_numeric_dtype(translation_z) and pd.api.types.is_numeric_dtype(frame):
        plt.plot(frame, translation_z, color = "blue")
        plt.plot(frame, -translation_z_2, color = "red")
        # Ajouter un titre et des labels
        plt.title('Translation Z vs Frame')
        plt.xlabel('Frame')
        plt.ylabel('Translation Z')
        plt.show(block=True)

        plt.figure(2)
        plt.plot(frame, translation_rx)
        plt.plot(frame, -translation_rx_2)
        # Ajouter un titre et des labels
        plt.title('Translation RX vs Frame')
        plt.xlabel('Frame')
        plt.ylabel('Translation RX')
        plt.show(block=True)

        plt.figure(3)
        plt.plot(frame, translation_ry)
        plt.plot(frame, -translation_ry_2)
        # Ajouter un titre et des labels
        plt.title('Translation RY vs Frame')
        plt.xlabel('Frame')
        plt.ylabel('Translation RY')
        plt.show(block=True)
    
    
        
    else:
        print("Les colonnes 'TZ' et 'Frame' doivent contenir des données numériques.")
else:
    print("Les colonnes 'TZ' et/ou 'Frame' ne sont pas présentes dans le fichier CSV.")


