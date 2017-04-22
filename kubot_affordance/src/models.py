import numpy as np
import math
import pickle
from copy import deepcopy

class ActionModel():
    def __init__(self, action, run_id,
                 feature_size=51,models_path='/home/cem/learning/models/',
                 epsilon_o=0.4, epsilon_r=0.0005, epsilon_e=0.08,
                 alpha_o=0.2, alpha_e=0.1, alpha_r=0.2, d_o=0.2, d_e=0.03,
                 t_alpha=1000.0, t_d=1000.0, t_r=1000.0):
        self.action = action
        self.feature_size = feature_size
        self.epsilon_o = epsilon_o
        self.epsilon_r = epsilon_r
        self.epsilon_e = epsilon_e
        self.alpha_o = alpha_o
        self.alpha_e = alpha_e
        self.alpha_r = alpha_r
        self.t_alpha = t_alpha #Decay rate of alpha of SOM
        self.t_d = t_d # Decay rate of d of SOM
        self.t_r = t_r # Decay rate of alpha of regression
        self.d_o = d_o
        self.d_e = d_e
        self.object_som = SOM(feature_size=self.feature_size-3,alpha0=self.alpha_o, d0=self.d_o, t_alpha=self.t_alpha, t_d=self.t_d)
        self.effect_som = SOM(feature_size=3,alpha0=self.alpha_e, d0=self.d_e, t_alpha=self.t_alpha, t_d=self.t_d)
        self.obj_model_map = {}
        self.models_path = models_path
        self.run_id = run_id
        self.path = '%s%s' % (self.models_path, self.action.name)
        self.e_som_path = '%s_effect_som_%d.pkl' % (self.path, self.run_id)
        self.b_som_path = '%s_before_som_%d.pkl' % (self.path, self.run_id)
        self.map_path = '%s_map_%d.pkl' % (self.path, self.run_id)
        self.before_scaler_path = '%s_before_scaler.pkl' % (self.path)
        self.effect_scaler_path = '%s_effect_scaler.pkl' % (self.path)

    def load(self):
        self.effect_som = pickle.load(open(self.e_som_path, "rb"))
        self.object_som = pickle.load(open(self.b_som_path, "rb"))
        self.obj_model_map = pickle.load(open(self.map_path, "rb"))

    def load_scalers(self):
        self.before_scaler = pickle.load(open(self.before_scaler_path, "rb"))
        self.effect_scaler = pickle.load(open(self.effect_scaler_path, "rb"))

    def update(self, before_feats, after_feats,is_gone):
        x_s = self.before_scaler.transform(before_feats.reshape(1,-1)).flatten()[3:51]
        if not is_gone:
            y = np.absolute(np.subtract(after_feats, before_feats))
            y_s = self.effect_scaler.transform(y.reshape(1,-1)).flatten()[0:3]
        else:
            y_s = self.effect_scaler.transform(before_feats.reshape(1,-1)).flatten()[0:3]

        o_min_distance = self.object_som.get_min_distance(x_s)
        if o_min_distance > self.epsilon_o or o_min_distance == -1:
            new_cid = self.object_som.add_neuron(x_s)
            self.obj_model_map[new_cid] = OnlineRegression(alpha0=self.alpha_r, t_r=self.t_r)

        o_cid = self.object_som.update(x_s)
        print "Picked regression model:",o_cid
        regressor = self.obj_model_map[o_cid]
        J = regressor.update(x_s, y_s)

        e_min_distance = self.effect_som.get_min_distance(y_s)
        if e_min_distance == -1 or e_min_distance > self.epsilon_e:
            self.effect_som.add_neuron(y_s)

        e_cid = self.effect_som.update(y_s)
        print "Picked effect cluster:", e_cid

        print "Effect som winner:", self.effect_som.winner(y_s)
        print "Before som #neurons:", len(self.object_som.weights)
        print "Effect som #neurons:", len(self.effect_som.weights)

        if J < self.epsilon_r:
            return True
        return False

    def predict(self, before_feats):
        x_s = self.scale(before_feats)
        obj_cid = self.object_som.winner(x_s)
        print "Object cid:", obj_cid
        gd = self.obj_model_map[obj_cid]
        y_predicted = gd.predict(x_s)
        effect_id = self.effect_som.winner(y_predicted.flatten())
        return effect_id, y_predicted

    def save(self):
        pickle.dump(self.effect_som, open(self.e_som_path, "wb"))
        pickle.dump(self.object_som, open(self.b_som_path, "wb"))
        pickle.dump(self.obj_model_map, open(self.map_path, "wb"))

class SOM():

    def __init__(self,feature_size=51, alpha0=0.2, d0=0.5, t_alpha=100, t_d=10):
        self.alpha0 = alpha0
        self.d0 = d0
        self.t_alpha = t_alpha
        self.t_d = t_d
        self.t = 0
        self.feature_size = feature_size
        self.weights = []

    def add_neuron(self, weight):
        self.weights.append(weight)
        return len(self.weights) - 1

    def decay_alpha(self):
        return self.alpha0 * np.exp(-1.0 * (float(self.t) / float(self.t_alpha)))

    def decay_d(self):
        return self.d0 * np.exp(-1.0 * (float(self.t) / float(self.t_d)))

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

    def __init__(self, dimensions = (3,49), alpha0 = 0.2, t_r = 100):
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
        print J
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
