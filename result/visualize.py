import csv
import matplotlib.pyplot as plt

radar_scores = []
lidar_scores = []

with open('radar_nis.csv', 'r') as fp:
    reader= csv.reader(fp)
    for line in reader:
        score = float(line[0])
        radar_scores.append(score)

with open('lidar_nis.csv', 'r') as fp:
    reader= csv.reader(fp)
    for line in reader:
        score = float(line[0])
        lidar_scores.append(score)

radar_chi_sq_plt = [7.815] * len(radar_scores)
lidar_chi_sq_plt = [5.991] * len(lidar_scores)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

ax1.plot(radar_scores)
ax1.plot(radar_chi_sq_plt)
ax1.set_title("NIS Score at 3 Degree Freedom Over Time (Radar)")
ax1.set_ylabel("NIS Score")
ax1.set_xlabel("Time")

ax2.plot(lidar_scores)
ax2.plot(lidar_chi_sq_plt)
ax2.set_title("NIS Score at 2 Degree Freedom Over Time (Lidar)")
ax2.set_ylabel("NIS Score")
ax2.set_xlabel("Time")

plt.tight_layout()
plt.show()