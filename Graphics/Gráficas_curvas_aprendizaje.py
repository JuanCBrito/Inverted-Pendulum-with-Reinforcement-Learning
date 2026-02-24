import numpy as np
import matplotlib.pyplot as plt
import re

def load_and_smooth(filename, window=5):
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
files  = [  "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_decreciente_005/history.txt",
            "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_final/history.txt",
            "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_decreciente_02/history.txt",
            "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_decreciente_03/history.txt",
            "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_decreciente_05/history.txt",
            "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_decreciente_07/history.txt"
        ]
labels = [r"$\alpha_{k} = 0.05$", r"$\alpha_{k} = 0.1$", r"$\alpha_{k} = 0.2$", 
          r"$\alpha_{k} = 0.3$", r"$\alpha_{k} = 0.5$", r"$\alpha_{k} = 0.7$"
          ]
colors = ["tab:olive", "tab:pink", "tab:cyan", "tab:blue", "tab:orange", "tab:green"]

# files  = [  "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_final/history.txt",
#             "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_1200_step/history.txt",
#             "Modelos_Finales_Figuras/Pendulo_1/Modelo_con_entropia_step_gradual/history.txt"
#         ]
# labels = ["Model 1: 400 obs", "Model 2: 1200 obs", "Model 3: gradual"]
# colors = ["tab:pink", "tab:gray", "tab:red"]

# files  = [  "notas/1500 epocas sin guardar modelo.txt",
#             "notas/Modelo_1_reward_scale_10.txt"
#         ]
# labels = ["Reward scale x2", "Reward scale x10"]
# colors = ["tab:blue", "tab:gray"]

plt.figure(figsize=(12,6))

for fname, label, c in zip(files, labels, colors):
    eps, scores, ep_ma, mean_ma, std_ma = load_and_smooth(fname)
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
# plt.title("Reward Scale Comparison")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.xlim(4, 790)
plt.ylim(-800, 500)
plt.savefig("Graficas/curvas_de_aprendizaje/Multiples_entropias_decrecientes_opcion_11.png")
plt.show()
