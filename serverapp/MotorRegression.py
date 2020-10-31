import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt


x1 = np.array([10, 20, 30, 40, 50, 60, 70, 80]).reshape((-1, 1))
x2 = np.array([10, 20, 30, 40, 50, 60]).reshape((-1, 1))
y1 = np.array([30, 81, 133, 182, 224, 274, 330, 406])
y2 = np.array([28, 73, 120, 164, 212, 254, 318, 390])
y3 = np.array([24, 64, 107, 147, 180, 206])

model1 = LinearRegression().fit(x1, y1)
print('intercept1:', model1.intercept_)
print('slope1:', model1.coef_)

model2 = LinearRegression().fit(x1, y2)
print('intercept2:', model2.intercept_)
print('slope2:', model2.coef_)

model3 = LinearRegression().fit(x2, y3)
print('intercept3:', model3.intercept_)
print('slope3:', model3.coef_)

plt.plot(x1, y1, 'r')
plt.plot(x1, y2, 'b')
plt.plot(x2, y3, 'g')
plt.show()

