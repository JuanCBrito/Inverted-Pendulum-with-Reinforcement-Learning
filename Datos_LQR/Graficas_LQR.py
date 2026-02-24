import pandas as pd
import matplotlib.pyplot as plt

def plot_episode_variables(t, pos_list, vel_list, ang_list, ang_vel_list, accion, figure_file="test"):
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



if __name__ == "__main__":
    ruta_csv = "D:\Juank\Pendulum_Inverted_SAC\Datos_LQR\LQR_Pendulo_1.csv"
    df = pd.read_csv(ruta_csv)
    t = df["Time"]
    ang_list = df["Angle"]
    ang_vel_list = df["Angular Velocity"]
    pos_list = df["Cart Position"]
    vel_list = df["Cart Speed"]
    accion = df["PWM"]

    plot_episode_variables(t, pos_list, vel_list, ang_list, ang_vel_list, accion)
    # plt.savefig(f"LQR.png", dpi=600)