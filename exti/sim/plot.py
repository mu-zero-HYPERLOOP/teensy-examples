import pandas as pd
from matplotlib import pyplot as plt

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

columns = ["time", "velocity", "distance", "left", "right", "estimated_distance", "estimated_velocity", "stripe_count", "isr_called", "ewma_distance", "distance_error"]


df = pd.read_csv("./result.csv", usecols=columns);
print("Contents in csv file: ", df)



figure, axis = plt.subplots(4);

axis[0].plot(df.time, df.distance, label="distance");
axis[0].plot(df.time, df.estimated_distance, label="estimation");
axis[0].plot(df.time, df.distance_error, label="error");
# axis[0].plot(df.time, df.ewma_distance, label="ewma_estimation");
axis[0].set_title("distance");
axis[0].grid();
axis[0].legend();

axis[1].plot(df.time, df.velocity, label="velocity");
axis[1].plot(df.time, df.estimated_velocity, label="estimation");
axis[1].set_title("velocity");
axis[1].grid();
axis[1].legend();



axis[2].plot(df.time, df.left, label="digital-left");
axis[2].plot(df.time, df.right, label="digital-right");
axis[2].set_title("digital readings");
axis[2].grid();
axis[2].legend();



axis[3].plot(df.time, df.stripe_count, label="stripe_count")
# axis[3].plot(df.time, df.isr_called, label="isr_called")
axis[3].set_title("stripe count");
axis[3].legend();
axis[3].grid();

plt.show();
