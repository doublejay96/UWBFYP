#OLS Regressor for State Space Model Estimation
import numpy as np
import statsmodels.api as sm
import matplotlib.pyplot as plt

curr_run = "run32manual"

#matrix is [time, PID_x_error, PID_y_error, PID_z_error, offboard_x_velocity, offboard_y_velocity, offboard_z_velocity]
super_logs = np.loadtxt(curr_run+'/super_logs.csv', delimiter=',',skiprows=1,usecols=(0,12,13,14,15,16,17))

X = super_logs[:-1,1:]#drop last row
Y_x = super_logs[1:,1]#drop first rows
Y_y = super_logs[1:,2]
Y_z = super_logs[1:,3]

model_x = sm.OLS(Y_x, X).fit()
model_y = sm.OLS(Y_y, X).fit()
model_z = sm.OLS(Y_z, X).fit()
model_x.summary()

coeff_matrix = np.vstack([model_x.params, model_y.params, model_z.params])
A = coeff_matrix[:,:3]
B = coeff_matrix[:,3:]
print("system matrix A")
print(A)
print("input matrix B")
print(B)

#predictions = model.predict(X) # make the predictions by the model
#model.summary()

#VICON_follower_x = super_logs[:,1] #in m
#VICON_follower_y = super_logs[:,2]
#VICON_follower_z = super_logs[:,3]
#offboard_input_vx = super_logs[:,4] # in m/s
#offboard_input_vy = super_logs[:,5]
#offboard_input_vz = super_logs[:,6]
