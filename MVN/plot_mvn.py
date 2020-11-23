import matplotlib.pyplot as plt
import csv
with open('/home/aseelam/Desktop/rand_matrix.csv',newline='') as csvfile:
    reader = csv.reader(csvfile)
    x_coordinates = []
    y_coordinates = []
    z_coordinates = []
    for row in reader:
        x_coordinates.append(float(row[0]))
        y_coordinates.append(float(row[1]))
        z_coordinates.append(float(row[2]))
    plt.scatter(x_coordinates,y_coordinates)
    plt.savefig('/home/aseelam/Desktop/mvn.png')