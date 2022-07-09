import numpy as np

# Ortvektoren
x1 = np.array([-1500,-1500,0])
x2 = np.array([-1500, 1500,0])
x3 = np.array([ 1500,-1500,0])
x4 = np.array([ 1500, 1500,0])

# Schwerpunkt
xm1 = np.array([ 100, 0, -100]) 

# Anziehungskraft
fm1 = np.array([ 0, 0, -0.7*9.8]) 

# Kräfte der Motoren
f1 = f2 = f3 = f4 = fm1/4

moment = np.cross(x1 - xm1, f1)
moment += np.cross(x2 - xm1, f2)
moment += np.cross(x3 - xm1, f3)
moment += np.cross(x4 - xm1, f4)

# Zylinder Trägheitstensor
r = 50
I = 1/2 * 0.7 * r * r

# Winkelbeschleunigung 
alpha = moment/I

print(moment)
print(alpha)

# Orientierung nach Zeit t
for t in range(0,10):
    orientation = alpha/2 * t * t *180/3.14
    print(orientation)

# x1 = np.array([10, 20, 30, 40, 50, 60, 70, 80]).reshape((-1, 1))
# x2 = np.array([10, 20, 30, 40, 50, 60]).reshape((-1, 1))
# # 6 Zoll, 12.37V - 12,2V
# y1 = np.array([30, 81, 133, 182, 224, 274, 330, 406])
# # 6 Zoll, 11,55V - 11,48V
# y2 = np.array([28, 73, 120, 164, 212, 254, 318, 390])
# # 5 Zoll, 11,48V - 11,48V
# y3 = np.array([24, 64, 107, 147, 180, 206])

# model1 = LinearRegression().fit(x1, y1)
# print('intercept1:', model1.intercept_)
# print('slope1:', model1.coef_)

# model2 = LinearRegression().fit(x1, y2)
# print('intercept2:', model2.intercept_)
# print('slope2:', model2.coef_)

# model3 = LinearRegression().fit(x2, y3)
# print('intercept3:', model3.intercept_)
# print('slope3:', model3.coef_)

# plt.plot(x1, y1, 'r')
# plt.plot(x1, y2, 'b')
# plt.plot(x2, y3, 'g')
# plt.show()

