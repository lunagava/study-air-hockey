import numpy as np

gt = np.genfromtxt("data.log", delimiter=" ", names=["line", "stamp", "x", "y", "evts"])
out_file = open("points.csv", "w")

for i in range(len(gt)):
    out_file.write(str(i)+" "+str(gt["stamp"][i])+" "+str(int(gt["x"][i]))+" "+str(int(gt["y"][i]))+" "+str(gt["evts"][i]*0.001)+ '\n')

