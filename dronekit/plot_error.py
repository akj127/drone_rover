x = []
y = []

with open("drone.pos.out", 'r') as file:
	for line in file:
		line = line.split("\t")
		x.append(float(line[0]))
		y.append(float(line[1]))

zeros = [0] * len(x)

import matplotlib.pyplot as plt

plt.plot(x, label="x")
plt.plot(y, label="y")
plt.plot(zeros, "--")

plt.legend()
plt.show()
