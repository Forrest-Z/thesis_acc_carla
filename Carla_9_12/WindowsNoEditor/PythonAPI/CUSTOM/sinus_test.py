import datetime
import math
import time
import matplotlib.pyplot as plt
dt = 0.033

max_time = 100
max_iter = math.ceil(max_time/dt)
print("Max iter:",max_iter)
y = []
t = []

start_time = time.time()

for i in range(max_iter):
    #time_val = time.time()-start_time
    time_val = dt*i
    t.append(time_val)
    y.append(math.sin(2*math.pi*0.01*time_val))
    pass

plt.plot(t,y)
plt.show()