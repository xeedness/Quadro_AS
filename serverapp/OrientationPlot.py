import numpy as np
import math

def main():
    v_base = np.array([[0],[0],[1]])
    print(v_base)
    ay = 0.1
    for ax in np.arange(0, math.pi, math.pi/8):
        rotY = np.array((
            (np.cos(ay), 0, np.sin(ay)), 
            (0, 1, 0), 
            (-np.sin(ay), 0, np.cos(ay))))

        rotX = np.array((
            (1, 0, 0), 
            (0, np.cos(-ax), -np.sin(-ax)), 
            (0, np.sin(-ax), np.cos(-ax))))

        v = np.dot(rotX, v_base)
        v = np.dot(rotY, v)
        
        print(v)
        ax_back = math.atan2(v[1], v[2])
        ay_back = math.atan2(v[0], v[2])
        print(ax, " == ",ax_back)
        print(ay, " == ",ay_back)




if __name__ == "__main__":
    main()