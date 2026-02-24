import numpy as np
import matplotlib.pyplot as plt
import re

episodes = []
scores = []
avg_scores = []

with open("notas/Modelo_1_reward_scale_10.txt", "r", encoding="utf-8") as f:
    for line in f:
        match = re.match(r"episode\s+(\d+)\s+score\s+([-\d.]+)\s+avg_score\s+([-\d.]+)", line)
        if match:
            episodes.append(int(match.group(1)))
            scores.append(float(match.group(2)))
            avg_scores.append(float(match.group(3)))
# 1) Opcional: calcular std móvil de los scores
window = 20  # ancho de la ventana para suavizar
scores_arr = np.array(scores)
rolling_mean = np.convolve(scores_arr, np.ones(window)/window, mode='valid')
rolling_std  = np.sqrt(
    np.convolve((scores_arr - np.mean(scores_arr))**2, 
                np.ones(window)/window, mode='valid')
)

# para alinear con episodes:
ep_ma = episodes[window-1:]

plt.figure(figsize=(12,6))

# 2) “Espectro” de scores como línea muy fina y transparente
plt.plot(episodes, scores, linewidth=0.2, alpha=0.2, label="Scores")

# 3) Curva promedio (ajustada o tu avg_scores)
plt.plot(ep_ma, rolling_mean, linewidth=2, label="Avg_score".format(window))

# 4) Banda de ± std
plt.fill_between(ep_ma,
                 rolling_mean - rolling_std,
                 rolling_mean + rolling_std,
                 alpha=0.3, 
                 label="± desviación")

plt.xlabel("Episode")
plt.ylabel("Score")
plt.title("Evolución del avg_score")
plt.legend()
plt.grid(True)
plt.savefig("notas/1500 epocas sin modelo scale_reward_10.png")
plt.show()
