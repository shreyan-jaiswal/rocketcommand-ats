import matplotlib.pyplot as plt
import os


varname = "ATS CPP/0p"

filename = varname + ".txt"

with open(filename) as f:
    content = f.readlines()

data = []

for line in content:
    for val in line.split(" "):
        data.append(float(val))

plt.plot(data)
plt.savefig(varname + ".png")
plt.close()
