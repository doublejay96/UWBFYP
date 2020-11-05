#A script that does analysis of the PDoA output in CSV file
import numpy as np
import matplotlib.pyplot as plt

def moving_average_filter(pdoa_data, col, length): #function to efficiently apply moving average of length onto col in pdoa_data
    sums = np.cumsum(pdoa_data[:,col])
    sums[length:] = sums[length:] - sums[:-length]
    return sums[length-1:] / length

def get_stddev_var_from_filter(pdoa_data): #function to apply each length of a moving-average filter on each value, store the statistics in an array
    statistics = np.zeros((4, 10, 2)) #first index value, second ind length, third ind std or var
    for col in range(4):
        for length in range(1,11):
            filtered_values = moving_average_filter(pdoa_data, col, length) #apply the moving average filter of length to col
            statistics[col,length-1,0] = np.std(filtered_values)
            statistics[col,length-1,1] = np.var(filtered_values)
            #print("For length {}, stddev is {}, var is {}".format(length, statistics[col,length,0], statistics[col,length,1]))
    return statistics

#CSV file columns are D, Xcm, Ycm, P (in deg)
pdoa_data = np.loadtxt("pdoa_output_100sec_60_10.csv",delimiter=',') #read the recorded pdoa data that was output to csv file
num_readings = pdoa_data.shape[0] #get the number of readings in the csv (for plotting later)
values_mean = np.mean(pdoa_data, axis=0)
values_var = np.var(pdoa_data, axis=0)
values_stddev = np.std(pdoa_data, axis=0)
print("Mean values for D, Xcm, Ycm, P:")
print(values_mean)
print("Variance values for D, Xcm, Ycm, P:")
print(values_var)
print("Standard deviation values for D, Xcm, Ycm, P:")
print(values_stddev)
statistics = get_stddev_var_from_filter(pdoa_data)
print("Statistics after applying various filters: indices are ((D,Xcm,Ycm,P), length from 1 to 10, (std,var))")
print(statistics)

fig1, axs1 = plt.subplots(2, 2)
axs1[0,0].scatter(np.arange(0, num_readings), pdoa_data[:,0])
axs1[0,0].title.set_text("Values of D")
axs1[0,0].set_ylabel("Value of D (cm)")
axs1[0,0].set_xlabel("Reading")
axs1[1,0].scatter(np.arange(0, num_readings), pdoa_data[:,1])
axs1[1,0].title.set_text("Values of Xcm")
axs1[1,0].set_ylabel("Value of Xcm (cm)")
axs1[1,0].set_xlabel("Reading")
axs1[1,1].scatter(np.arange(0, num_readings), pdoa_data[:,2])
axs1[1,1].title.set_text("Values of Ycm")
axs1[1,1].set_ylabel("Value of Ycm (cm)")
axs1[1,1].set_xlabel("Reading")
axs1[0,1].scatter(np.arange(0, num_readings), pdoa_data[:,3])
axs1[0,1].title.set_text("Values of P")
axs1[0,1].set_ylabel("Value of P (degrees)")
axs1[0,1].set_xlabel("Reading")
plt.show()

fig2, axs2 = plt.subplots(2,2)
axs2[0,0].plot(np.arange(1, 11), statistics[0,:,0],label="Standard Deviation")
axs2[0,0].plot(np.arange(1, 11), statistics[0,:,1],label="Variance")
axs2[0,0].title.set_text("Standard Deviation and Variance of D against Length of Moving-Average Filter")
axs2[0,0].set_ylabel("Standard Deviation or Variance of D (cm)")
axs2[0,0].set_xlabel("Length of Moving-Average Filter")
axs2[0,0].set_xticks(np.arange(1,11))
axs2[0,0].legend()
axs2[1,0].plot(np.arange(1, 11), statistics[1,:,0],label="Standard Deviation")
axs2[1,0].plot(np.arange(1, 11), statistics[1,:,1],label="Variance")
axs2[1,0].title.set_text("Standard Deviation and Variance of Xcm against Length of Moving-Average Filter")
axs2[1,0].set_ylabel("Standard Deviation or Variance of Xcm (cm)")
axs2[1,0].set_xlabel("Length of Moving-Average Filter")
axs2[1,0].set_xticks(np.arange(1,11))
axs2[1,0].legend()
axs2[1,1].plot(np.arange(1, 11), statistics[2,:,0],label="Standard Deviation")
axs2[1,1].plot(np.arange(1, 11), statistics[2,:,1],label="Variance")
axs2[1,1].title.set_text("Standard Deviation and Variance of Ycm against Length of Moving-Average Filter")
axs2[1,1].set_ylabel("Standard Deviation or Variance of Ycm (cm)")
axs2[1,1].set_xlabel("Length of Moving-Average Filter")
axs2[1,1].set_xticks(np.arange(1,11))
axs2[1,1].legend()
axs2[0,1].plot(np.arange(1, 11), statistics[3,:,0],label="Standard Deviation")
axs2[0,1].plot(np.arange(1, 11), statistics[3,:,1],label="Variance")
axs2[0,1].title.set_text("Standard Deviation and Variance of P against Length of Moving-Average Filter")
axs2[0,1].set_ylabel("Standard Deviation or Variance of P (degrees)")
axs2[0,1].set_xlabel("Length of Moving-Average Filter")
axs2[0,1].set_xticks(np.arange(1,11))
axs2[0,1].legend()
plt.show()
