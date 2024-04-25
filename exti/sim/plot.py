import pandas as pd
from matplotlib import pyplot as plt

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

df = pd.read_csv("./result.csv");
print("Contents in csv file: ", df)

figure, axis = plt.subplots(2);

axis[0].plot(df.time, df.true_distance, label="distance");
axis[0].plot(df.time, df.d_linenc, label="linenc");
axis[0].plot(df.time, df.d_kalman, label="kalman");
axis[0].plot(df.time, df.d_error, label="error");
axis[0].set_title("distance");
axis[0].grid();
axis[0].legend();

axis[1].plot(df.time, df.true_velocity, label="velocity");
axis[1].set_title("velocity");
axis[1].grid();
axis[1].legend();

plt.show();
