import csv
import matplotlib.pyplot as plt

time = []
ref_x = []
ref_y = []
sol_x = []
sol_y = []

with open('results.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    
    next(csvreader)
    
    for row in csvreader:
        time.append(float(row[0]))
        ref_x.append(float(row[1]))
        ref_y.append(float(row[2]))
        sol_x.append(float(row[3]))
        sol_y.append(float(row[4]))

plt.figure(figsize=(8, 6))

plt.plot(ref_x, ref_y, 'ro-', label='Reference Path', markersize=5)

plt.plot(sol_x, sol_y, 'bo-', label='Actual Path', markersize=5)

plt.title('Reference Path vs Actual Path')
plt.xlabel('X Position')
plt.ylabel('Y Position')

plt.legend()

plt.grid(True)

plt.show()

