from time import time
import numpy as np
from collections import deque


class Queue:
    def __init__(self, window_size):
        self.window_size = window_size
        self.c = 0.25
        self.g = 9.81
        self.time = deque(np.ones(window_size)*time())
        self.tk = self.time[0]
        self.var = 0.025
        self.xp = np.zeros([2, 1])
        self.yp = np.zeros([2, 1])
        self.xp_queue = deque(np.zeros([window_size, 2]))
        self.yp_queue = deque(np.zeros([window_size, 2]))
        self.x_hat = np.zeros([2, 1])
        self.y_hat = np.zeros([2, 1])
        self.dx = np.zeros([2, 1])
        self.dy = np.zeros([2, 1])
        self.xtilde = deque(np.zeros(window_size))
        self.ytilde = deque(np.zeros(window_size))
        self.p = deque(np.ones([window_size, 2]) * 0.025)
        self.size = 0
        self.uwb = np.zeros(4)
        self.uwb_ready = np.zeros(4)

    def predict(self, eta, dt):
        pitch = eta[1] * np.pi / 180
        roll = eta[0] * np.pi / 180
        A = np.array([[1, dt],
                      [0, 1-self.c*dt]])
        B = np.array([[0], [-self.g*dt]])
        self.xp = A @ self.xp + B * np.tan(roll)
        self.yp = A @ self.yp + B * np.tan(pitch)
        return self.xp, self.yp

    def update(self, dt):
        self.x_hat = self.xp - np.array([[1, dt], [0, 1]]) @ self.dx
        self.y_hat = self.yp - np.array([[1, dt], [0, 1]]) @ self.dy
        return self.x_hat, self.y_hat

    def least_squares(self):
        N = self.window_size
        n_eqs = np.size(self.xtilde, 1)

        X = np.ones([N, 2])
        for i in range(len(self.time)):
            X[i, 1] = self.time[i] - self.time[0]

        Y = np.zeros([n_eqs * N, 1])
        Yx = Y.copy()
        Yy = Y.copy()
        Xx = X.copy()
        Xy = X.copy()

        for i in range(N):
            Yx[i, 0] = self.xp_queue[i][0] - self.xtilde[i]
            Yy[i, 0] = self.yp_queue[i][0] - self.ytilde[i]

        Q = np.array([[0, 0],
                      [0, 0.01]])

        self.dx = np.linalg.inv(Xx.T @ Xx + Q) @ (Xx.T @ Yx + Q @ self.dx)
        self.dy = np.linalg.inv(Xy.T @ Xy + Q) @ (Xy.T @ Yy + Q @ self.dy)


    def new_meas(self):
        self.xtilde.rotate(-1)
        self.ytilde.rotate(-1)

        # calculate new measurements
        xtilde, ytilde = self.meas_to_tilde()
        self.xtilde[-1] = xtilde
        self.ytilde[-1] = ytilde

        self.xp_queue.rotate(-1)
        self.xp_queue[-1] = self.xp
        self.yp_queue.rotate(-1)
        self.yp_queue[-1] = self.yp
        self.time.rotate(-1)
        self.time[-1] = time()
        self.tk = self.time[0]

        self.size += 1

        if self.size >= self.window_size:
            self.least_squares()

    def meas_to_tilde(self):
        z = 2
        d1 = self.uwb[0]**2
        d2 = self.uwb[1]**2
        d3 = self.uwb[2]**2
        d4 = self.uwb[3]**2
        x1 = -4.715
        x2 = -4.819
        x3 = 4.902
        x4 = 4.799
        y1 = -4.539
        y2 = 4.965
        y3 = 5.029
        y4 = -4.606
        z1 = 1.6
        z2 = 1.66
        z3 = 0.97
        z4 = 1.6

        A = np.array([[1, -2 * x1, -2 * y1],
                      [1, -2 * x2, -2 * y2],
                      [1, -2 * x3, -2 * y3],
                      [1, -2 * x4, -2 * y4]])

        V = np.array([[4 * d1 * 0.025 + 2 * 0.025 * 0.025, 0, 0, 0],
                      [0, 4 * d2 * 0.025 + 2 * 0.025 * 0.025, 0, 0],
                      [0, 0, 4 * d3 * 0.025 + 2 * 0.025 * 0.025, 0],
                      [0, 0, 0, 4 * d4 * 0.025 + 2 * 0.025 * 0.025]])

        b = np.array([[d1 - x1 ** 2 - y1 ** 2 - z1 ** 2 - z ** 2 + 2 * z * z1],
                      [d2 - x2 ** 2 - y2 ** 2 - z2 ** 2 - z ** 2 + 2 * z * z2],
                      [d3 - x3 ** 2 - y3 ** 2 - z3 ** 2 - z ** 2 + 2 * z * z3],
                      [d4 - x4 ** 2 - y4 ** 2 - z4 ** 2 - z ** 2 + 2 * z * z4]])

        Vinv = np.linalg.inv(V)
        pos = np.linalg.inv(A.T  @ Vinv @ A) @ A.T @ Vinv @ b
        x = pos[1]
        y = pos[2]

        return x, y
