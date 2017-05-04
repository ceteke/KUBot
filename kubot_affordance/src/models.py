import numpy as np
import math
import pickle
from copy import deepcopy
from keras.models import load_model
from keras.models import Sequential
from keras.layers import Dense, Dropout

class ActionModel():
    def __init__(self, action, run_id,
                 feature_size=51,models_path='/home/cem/learning/models/',
                 epsilon_r=0.01, epsilon_e=0.08,
                 alpha_e=0.1, d_e=0.03,
                 t_alpha=100.0, t_d=100.0):
        self.action = action
        self.feature_size = feature_size
        self.epsilon_r = epsilon_r
        self.epsilon_e = epsilon_e
        self.alpha_e = alpha_e
        self.t_alpha = t_alpha #Decay rate of alpha of SOM
        self.t_d = t_d # Decay rate of d of SOM
        self.d_e = d_e
        self.nn = None
        self.effect_som = SOM(feature_size=3,alpha0=self.alpha_e, d0=self.d_e, T1=self.t_alpha, T2=self.t_d)
        self.models_path = models_path
        self.run_id = run_id
        self.path = '%s%s' % (self.models_path, self.action.name)
        self.e_som_path = '%s_effect_som_%d.pkl' % (self.path, self.run_id)
        self.nn_path = '%s_nn_%d.h5' % (self.path, self.run_id)
        self.before_scaler_path = '%sobject_scaler.pkl' % (self.models_path)
        self.effect_scaler_path = '%seffect_scaler.pkl' % (self.models_path)

    def load(self):
        self.effect_som = pickle.load(open(self.e_som_path, "rb"))
        self.nn = load_model(self.nn_path)

    def load_scalers(self):
        self.before_scaler = pickle.load(open(self.before_scaler_path, "rb"))
        self.effect_scaler = pickle.load(open(self.effect_scaler_path, "rb"))

    def init_models(self):
        self.nn = Sequential()
        self.nn.add(Dense(128, input_dim=51, activation='relu'))
        self.nn.add(Dense(128, activation='relu'))
        self.nn.add(Dense(64, activation='relu'))
        self.nn.add(Dense(3, activation='relu'))
        self.nn.compile(loss='mean_absolute_error', optimizer='adagrad')

    def update(self, before_feats, after_feats):
        x_s = self.before_scaler.transform(before_feats.reshape(1,-1)).flatten()
        
        y = np.absolute(np.subtract(after_feats, before_feats))
        y_s = self.effect_scaler.transform(y.reshape(1,-1)).flatten()[0:3]

        y_predicted = self.nn.predict(x_s.reshape(1,-1))

        hist = self.nn.fit(x_s.reshape(1,-1), y_s.reshape(1,-1), batch_size=1, epochs=10, verbose=0)
        loss = hist.history['loss'][-1]
        print "Training loss:", loss
        if loss < self.epsilon_r:
            return True

        e_min_distance = self.effect_som.get_min_distance(y_s)
        if e_min_distance == -1 or e_min_distance > self.epsilon_e:
            e_cid = self.effect_som.add_neuron(y_s)
        else:
            e_cid = self.effect_som.update(y_s)

        print "Effect som cluster:", e_cid
        print "Effect som #neurons:", len(self.effect_som.weights)

        return False

    def predict(self, before_feats):
        x_s = self.before_scaler.transform(before_feats.reshape(1,-1)).flatten()
        y_predicted = self.nn.predict(x_s.reshape(1,-1)).flatten()
        effect_id = self.effect_som.winner(y_predicted)
        return effect_id, y_predicted

    def save(self):
        pickle.dump(self.effect_som, open(self.e_som_path, "wb"))
        self.nn.save(self.nn_path)

class SOM():

    def __init__(self,feature_size=51, alpha0=0.2, d0=0.5, T1=1000, T2=100):
        self.alpha0 = alpha0
        self.d0 = d0
        self.T1 = T1
        self.T2 = T2
        self.t = 0
        self.feature_size = feature_size
        self.weights = []

    def add_neuron(self, weight):
        self.weights.append(weight)
        return len(self.weights) - 1

    def decay_alpha(self):
        return self.alpha0 * np.exp(-1.0 * (float(self.t) / float(self.T1)))

    def decay_d(self):
        return self.d0 * np.exp(-1.0 * (float(self.t) / float(self.T2)))

    def get_bmu_index(self, x):
        diff = [np.linalg.norm(x - w) for w in self.weights]
        return np.argmin(diff)

    def get_min_distance(self, x):
        if len(self.weights) == 0:
            return -1
        diff = [np.linalg.norm(x - w) for w in self.weights]
        return np.min(diff)

    def winner(self, x):
        return self.get_bmu_index(x)

    def neighborhood(self, j, i):
        d = self.decay_d()
        weight_distance = np.linalg.norm(self.weights[i] - self.weights[j])
        if weight_distance > d:
            return 0.0
        return 1.0

    def update(self, x):
        i = self.get_bmu_index(x)
        for j in range(len(self.weights)):
            neighborhood = self.neighborhood(j, i)
            self.weights[j] = self.weights[j] + self.decay_alpha() * neighborhood * (x - self.weights[j])
        self.t += 1
        return i

class OnlineRegression():

    def __init__(self, dimensions = (3,4), alpha0 = 0.2, t_r = 100):
        self.name = 'online_regression'
        self.dimensions = dimensions
        self.alpha0 = alpha0
        self.W = np.random.rand(self.dimensions[0], self.dimensions[1])
        self.Js = []
        self.t = 0
        self.t_r = float(t_r)

    def get_mean_err(self):
        return np.mean(self.Js)

    def decay_alpha(self):
        return self.alpha0 * np.exp(-1 * (float(self.t) / float(self.t_r)))

    def update(self, x, y):
        x_s = self.__preproc_x(x)
        y_s = y[np.newaxis].T
        #y_s = np.vstack([y_s, [0.0]])
        J = self.get_square_error(x_s, y_s) / 2
        #print J
        self.Js.append(J)
        dJdW = np.matmul(self.W, np.matmul(x_s, x_s.T)) - np.matmul(y_s, x_s.T)
        self.W -= self.decay_alpha() * dJdW
        self.t += 1
        return J

    def get_square_error(self, x, y):
        a = y - np.matmul(self.W, x)
        return np.matmul(a.T, a)[0][0]

    def predict(self, x):
        x_s = self.__preproc_x(x)
        return np.matmul(self.W, x_s).flatten()

    def __preproc_x(self, x):
        x_s = x[np.newaxis].T
        x_s = np.vstack([x_s, [1.0]])
        return x_s
