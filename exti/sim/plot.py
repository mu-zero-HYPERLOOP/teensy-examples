import pandas as pd
from matplotlib import pyplot as plt

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

df = pd.read_csv("./result.csv");
print("Contents in csv file: \n", df)
print(df.info())

figure, axis = plt.subplots(3);

axis[0].plot(df.time, df.s_true, label="distance");
axis[0].scatter(df.time, df.s_read, label="measurement", s=5, c='red');
axis[0].plot(df.time, df.s_kalman, label="kalman");
axis[0].plot(df.time, df.s_linenc, label="linear encoder");
axis[0].set_title("distance");
axis[0].grid();
axis[0].legend();

axis[1].plot(df.time, df.v_true, label="velocity");
axis[1].plot(df.time, df.v_kalman, label="kalman");
axis[1].set_title("velocity");
axis[1].grid();
axis[1].legend();

axis[2].plot(df.time, df.a_true, label="acceleration");
axis[2].scatter(df.time, df.a_read, label="measurement", s=5, c='red');
axis[2].plot(df.time, df.a_kalman, label="kalman");
axis[2].set_title("acceleration");
axis[2].grid();
axis[2].legend();

plt.show();
