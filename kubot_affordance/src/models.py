import numpy as np
import math
import pickle

class GradientDescent():

    def __init__(self, scaler, dimensions = 70, alpha0 = 0.2):
        self.name = 'gradient_descent'
        self.dimensions = dimensions
        self.alpha0 = alpha0
        self.W = np.random.rand(self.dimensions, self.dimensions)
        self.Js = []
        self.t = 0
        self.scaler = scaler
        self.alpha_t = self.alpha0

    def update(self, x, y, epsilon):
        x_s = self.__preproc_x(x)
        y_s = self.scaler(y)
        y_s = y_s[np.newaxis].T
        y_s = np.vstack([y_s, [0.0]])
        J = self.get_square_error(x_s, y_s) / 2
        print J
        if J <= epsilon:
            return False
        self.Js.append(J)
        dJdW = np.matmul(self.W, np.matmul(x_s, x_s.T)) - np.matmul(y_s, x_s.T)
        self.W -= self.alpha_t * dJdW
        alpha_t = self.alpha0*500/(self.t+500)
        self.t += 1
        return True

    def get_square_error(self, x, y):
        a = y - np.matmul(self.W, x)
        return np.matmul(a.T, a)[0][0]

    def predict(self, x):
        x_s = self.__preproc_x(x)
        return np.delete(np.matmul(self.W, x_s), self.dimensions-1, 0)

    def __preproc_x(self, x):
        x_s = self.scaler(x)
        x_s = x_s[np.newaxis].T
        x_s = np.vstack([x_s, [1.0]])
        return x_s

    def get_rmse(self, X, y):
        total = 0.0
        c = 0.0
        for i in range(len(X)):
            x = X[i]
            y_s = self.scaler(y[i])
            y_s = y_s[np.newaxis].T
            y_predicted = self.predict(x)
            a = y_s - y_predicted
            err = np.matmul(a.T, a)[0][0]
            total += err
            c += 1.0
        return math.sqrt(total / c)
