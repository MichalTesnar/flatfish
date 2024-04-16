import pandas as pd
import matplotlib.pyplot as plt

# for each column show mean and variance of values
# for name in names:
#     print(f'{name}: mean={data[name].mean()}, variance={data[name].var()}')

# plot column with name time on x and linear_x on y in matplot lib
# plt.scatter(data['time'], data['linear_x'])
# plt.xlabel('time')
# plt.ylabel('linear_x')
# plt.show()

def plot_histogram(data):
    # calculate differences in time stamps
    time_diff = data['time'].diff()
    # # discard if greater two
    time_diff = time_diff[time_diff < 2]
    # time_diff = time_diff[time_diff < 0.13]
    
    # # big of a proportion of the dataset is discarded
    print(f'{len(time_diff) / len(data)}')
    # 
    # plot their histogram
    plt.hist(time_diff, bins=100)
    plt.xlabel('time difference')
    plt.ylabel('count')
    plt.show()

def plot_pair_plot(data):
    # take only the rows of data that are in the time_diff
    time_diff = data['time'].diff()
    data = data.iloc[time_diff.index]
    data = data.drop(columns=['time'])
    pd.plotting.scatter_matrix(data, alpha=0.2)
    plt.show()

# add main function
def main():
    data = pd.read_csv('rosbag2_long_gathered.csv')
    # plot_pair_plot(data)
    plot_histogram(data)

if __name__ == '__main__':
    main()