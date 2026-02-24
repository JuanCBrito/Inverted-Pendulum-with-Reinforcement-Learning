import pandas as pd
import matplotlib.pyplot as plt

def plot_episode_variables(t, pos_list, vel_list, ang_list, ang_vel_list, accion, figure_file="None"):
    plt.figure(figsize=(12,8))
    plt.subplot(3,2,1)
    plt.plot(t, pos_list)
    plt.title("Posición del carrito")
    plt.xlabel("Tiempo (steps)")
    plt.ylabel("Posición")

    plt.subplot(3,2,2)
    plt.plot(t, vel_list)
    plt.title("Velocidad del carrito")
    plt.xlabel("Tiempo (steps)")
    plt.ylabel("Velocidad")

    plt.subplot(3,2,3)
    plt.plot(t, ang_list)
    plt.title("Ángulo del péndulo")
    plt.xlabel("Tiempo (steps)")
    plt.ylabel("Ángulo (grados)")

    plt.subplot(3,2,4)
    plt.plot(t, ang_vel_list)
    plt.title("Velocidad angular del péndulo")
    plt.xlabel("Tiempo (steps)")
    plt.ylabel("Velocidad angular")

    plt.subplot(3,2,5)
    plt.plot(t, accion)
    plt.title("Señal de control")
    plt.xlabel("Tiempo (steps)")
    plt.ylabel("PWM")

    plt.tight_layout()
    plt.savefig(figure_file, dpi=600)
    plt.show()

def plot_comparative_episode_variables_3(
    t1, pos1, vel1, ang1, ang_vel1, accion1,
    t2, pos2, vel2, ang2, ang_vel2, accion2,
    t3, pos3, vel3, ang3, ang_vel3, accion3,
    labels=("None", "None", "None"),
    colors=("None", "None", "None"),
    figure_file="comparativa",
    linewidths=((1.5, 1.5, 1.35), (1.2, 1.2, 1.08), (1.5, 1.5, 1.35), (1.5, 1.5, 1.35), (1, 1, 0.9))
):
    fig = plt.figure(figsize=(12,8))
    plt.subplot(3,2,1)
    l1, = plt.plot(t1, pos1/25000, label=labels[0], color=colors[0], linewidth=linewidths[0][0])
    l2, = plt.plot(t2, pos2/25000, label=labels[1], color=colors[1], linewidth=linewidths[0][1])
    l3, = plt.plot(t3, pos3/25000, label=labels[2], color=colors[2], linewidth=linewidths[0][2])
    plt.title("Car Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    plt.subplot(3,2,2)
    plt.plot(t1, vel1/25000, color=colors[0], linewidth=linewidths[1][0])
    plt.plot(t2, vel2/25000, color=colors[1], linewidth=linewidths[1][1])
    plt.plot(t3, vel3/25000, color=colors[2], linewidth=linewidths[1][2])
    plt.title("Car Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")

    plt.subplot(3,2,3)
    plt.plot(t1, ang1, color=colors[0], linewidth=linewidths[2][0])
    plt.plot(t2, ang2, color=colors[1], linewidth=linewidths[2][1])
    plt.plot(t3, ang3, color=colors[2], linewidth=linewidths[2][2])
    plt.title("Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")

    plt.subplot(3,2,4)
    plt.plot(t1, ang_vel1, color=colors[0], linewidth=linewidths[3][0])
    plt.plot(t2, ang_vel2, color=colors[1], linewidth=linewidths[3][1])
    plt.plot(t3, ang_vel3, color=colors[2], linewidth=linewidths[3][2])
    plt.title("Angular Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity")

    plt.subplot(3,2,(5,6))
    plt.plot(t1, accion1*100, color=colors[0], linewidth=linewidths[4][0])
    plt.plot(t2, accion2*100, color=colors[1], linewidth=linewidths[4][1])
    plt.plot(t3, accion3*100, color=colors[2], linewidth=linewidths[4][2])
    plt.title("Control Signal")
    plt.xlabel("Time (s)")
    plt.ylabel("PWM (%)")

    # Leyenda general
    fig.legend([l1, l2, l3], labels, loc='upper center', ncol=3, fontsize=12)

    plt.tight_layout(rect=[0,0,1,0.96])
    plt.savefig(figure_file, dpi=600)
    plt.show()
    
def plot_comparative_episode_variables_2(
    t1, pos1, vel1, ang1, ang_vel1, accion1,
    t2, pos2, vel2, ang2, ang_vel2, accion2,
    labels=("None", "None"),
    colors=("None", "None"),
    figure_file="comparativa",
    linewidths=((1.5, 1.5), (1.2, 1.2), (1.5, 1.5), (1.5, 1.5), (1, 1))
):
    fig = plt.figure(figsize=(12,8))
    plt.subplot(3,2,1)
    l1, = plt.plot(t1, pos1/25000, label=labels[0], color=colors[0], linewidth=linewidths[0][0])
    l2, = plt.plot(t2, pos2/25000, label=labels[1], color=colors[1], linewidth=linewidths[0][1])
    plt.title("Car Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    plt.subplot(3,2,2)
    plt.plot(t1, vel1/25000, color=colors[0], linewidth=linewidths[1][0])
    plt.plot(t2, vel2/25000, color=colors[1], linewidth=linewidths[1][1])
    plt.title("Car Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")

    plt.subplot(3,2,3)
    plt.plot(t1, ang1, color=colors[0], linewidth=linewidths[2][0])
    plt.plot(t2, ang2, color=colors[1], linewidth=linewidths[2][1])
    plt.title("Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")

    plt.subplot(3,2,4)
    plt.plot(t1, ang_vel1, color=colors[0], linewidth=linewidths[3][0])
    plt.plot(t2, ang_vel2, color=colors[1], linewidth=linewidths[3][1])
    plt.title("Angular Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity")

    plt.subplot(3,2,(5,6))
    plt.plot(t1, accion1*10, color=colors[0], linewidth=linewidths[4][0])
    plt.plot(t2, accion2*100, color=colors[1], linewidth=linewidths[4][1])
    plt.title("Control Signal")
    plt.xlabel("Time (s)")
    plt.ylabel("PWM (%)")

    # Leyenda general
    fig.legend([l1, l2], labels, loc='upper center', ncol=2, fontsize=12)

    plt.tight_layout(rect=[0,0,1,0.96])
    plt.savefig(figure_file, dpi=600)
    plt.show()

# if __name__ == "__main__":
#     ruta_csv1 = "D:/Juank/Pendulum_Inverted_SAC/Graficas/Entrenamiento_csv_pendulo_1/sac_3_005.csv"
#     ruta_csv2 = "D:/Juank/Pendulum_Inverted_SAC/Graficas/Entrenamiento_csv_pendulo_1/sac_1_final.csv"
#     ruta_csv3 = "D:/Juank/Pendulum_Inverted_SAC/Graficas/Entrenamiento_csv_pendulo_1/sac_1_02.csv"

#     df1 = pd.read_csv(ruta_csv1)
#     df2 = pd.read_csv(ruta_csv2)
#     df3 = pd.read_csv(ruta_csv3)
    
#     t1 = df1["Tiempo"]
#     ang_list1 = df1["Ángulo"]
#     ang_vel_list1 = df1["Velocidad angular"]
#     pos_list1 = df1["Posición"]
#     vel_list1 = df1["Velocidad"]
#     accion1 = df1["Acción"]

#     t2 = df2["Tiempo"]
#     ang_list2 = df2["Ángulo"]
#     ang_vel_list2 = df2["Velocidad angular"]
#     pos_list2 = df2["Posición"]
#     vel_list2 = df2["Velocidad"]
#     accion2 = df2["Acción"]

#     t3 = df3["Tiempo"]
#     ang_list3 = df3["Ángulo"]
#     ang_vel_list3 = df3["Velocidad angular"]
#     pos_list3 = df3["Posición"]
#     vel_list3 = df3["Velocidad"]
#     accion3 = df3["Acción"]

#     plot_comparative_episode_variables_3(
#         t1, pos_list1, vel_list1, ang_list1, ang_vel_list1, accion1,
#         t2, pos_list2, vel_list2, ang_list2, ang_vel_list2, accion2,
#         t3, pos_list3, vel_list3, ang_list3, ang_vel_list3, accion3,
#         labels=("Entropía decreciente 0.05", "Entropía decreciente 0.1", "Entropía decreciente 0.2"),
#         colors=("tab:olive", "tab:pink" , "tab:cyan"),  # Puedes modificar estos colores
#         figure_file="Graficas/pruebas_de_control/control_variacion_entropia_decreciente_1.png",
#     )

if __name__ == "__main__":
    ruta_csv1 = "./Datos_LQR/LQR_Pendulo_1_2.csv"
    ruta_csv2 = "D:/Juank/Pendulum_Inverted_SAC/Graficas/Entrenamiento_csv_pendulo_1/sac_1_final.csv"

    df1 = pd.read_csv(ruta_csv1)
    df2 = pd.read_csv(ruta_csv2)
    
    t1 = df1["Tiempo"]
    ang_list1 = df1["Ángulo"]
    ang_vel_list1 = df1["Velocidad angular"]
    pos_list1 = df1["Posición"]
    vel_list1 = df1["Velocidad"]
    accion1 = df1["Acción"]

    t2 = df2["Tiempo"]
    ang_list2 = df2["Ángulo"]
    ang_vel_list2 = df2["Velocidad angular"]
    pos_list2 = df2["Posición"]
    vel_list2 = df2["Velocidad"]
    accion2 = df2["Acción"]


    plot_comparative_episode_variables_2(
        t1, pos_list1, vel_list1, ang_list1, ang_vel_list1, accion1,
        t2, pos_list2, vel_list2, ang_list2, ang_vel_list2, accion2,
        labels=("LQR", "SAC entropía 0.1"),
        colors=("tab:blue", "tab:pink"),  # Puedes modificar estos colores
        figure_file="Graficas/pruebas_de_control/lqr_vs_entropy_final.png",
    )