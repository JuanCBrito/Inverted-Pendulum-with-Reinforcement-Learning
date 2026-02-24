import numpy as np
import matplotlib.pyplot as plt
import re

def load_and_smooth(filename, window=20):
    episodes, scores = [], []
    with open(filename, "r", encoding="utf-8") as f:
        for line in f:
            m = re.match(r"episode\s+(\d+)\s+score\s+([-\d.]+)\s+avg_score\s+([-\d.]+)", line)
            if not m: continue
            episodes.append(int(m.group(1)))
            scores.append(float(m.group(2)))
    scores = np.array(scores)
    # media móvil y desviación
    kernel = np.ones(window) / window
    rolling_mean = np.convolve(scores, kernel, mode='valid')
    rolling_std  = np.sqrt(np.convolve((scores - scores.mean())**2, kernel, mode='valid'))
    # alinear episodios
    ep_ma = episodes[window-1:]
    return np.array(episodes), scores, ep_ma, rolling_mean, rolling_std

# lista de ficheros y etiquetas
files  = ["notas/Modelo_1_reward_scale_10.txt",
          "notas/1500 epocas sin guardar modelo.txt"]
labels = ["Modelo 1 (scale=10)", "1500 épocas sin guardar modelo"]
colors = ["C0", "C1"]

plt.figure(figsize=(12,6))

for fname, label, c in zip(files, labels, colors):
    eps, scores, ep_ma, mean_ma, std_ma = load_and_smooth(fname, window=20)
    # nube de scores
    plt.plot(eps, scores, linewidth=0.2, alpha=0.15, color=c)
    # media móvil
    plt.plot(ep_ma, mean_ma, linewidth=2, label=label, color=c)
    # banda ± std
    plt.fill_between(ep_ma,
                     mean_ma - std_ma,
                     mean_ma + std_ma,
                     alpha=0.2,
                     color=c)

plt.xlabel("Episode")
plt.ylabel("Score")
plt.title("Comparación de entrenamiento")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("notas/comparativa_reward_scale_vs_sin_guardar.png")
plt.show()
