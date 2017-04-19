import numpy as np
import math
import pickle
from sklearn.preprocessing import minmax_scale

class ActionModel():
    def __init__(self, action, run_id,
                 feature_size=51,models_path='/home/cem/learning/models/',
                 epsilon_o=0.01, epsilon_r=0.1, epsilon_e=0.014):
        self.action = action
        self.feature_size = feature_size
        self.object_som = SOM(feature_size=self.feature_size,alpha0=0.2, d0=0.005)
        self.effect_som = SOM(feature_size=self.feature_size,alpha0=0.2, d0=0.0075)
        self.obj_model_map = {}
        self.models_path = models_path
        self.run_id = run_id
        self.path = '%s%s' % (self.models_path, self.action.name)
        self.e_som_path = '%s_effect_som_%d.pkl' % (self.path, self.run_id)
        self.b_som_path = '%s_before_som_%d.pkl' % (self.path, self.run_id)
        self.map_path = '%s_map_%d.pkl' % (self.path, self.run_id)

    def load(self):
        self.effect_som = pickle.load(open(self.e_som_path, "rb"))
        self.object_som = pickle.load(open(self.b_som_path, "rb"))
        self.obj_model_map = pickle.load(open(self.map_path, "rb"))

    def scale(self, features):
        f_s = minmax_scale(features)
        position = f_s[0:3]
        others = f_s[3:self.feature_size]
        others = np.multiply(others, 0.01)
        new_feats = np.append(position, others)
        return new_feats

    def update(self, before_feats, after_feats,is_gone):
        x_s = self.scale(before_feats)
        if not is_gone:
            y = np.absolute(np.subtract(after_feats, before_feats))
            y_s = self.scale(y)
        else:
            y_s = np.array([-1.0] * self.feature_size)
        o_min_distance = self.object_som.get_min_distance(x_s)
        print "Before som min distance:", o_min_distance
        if o_min_distance > self.epsilon_o or o_min_distance == -1:
            self.object_som.add_neuron(x_s)
        else:
            self.object_som.update(x_s)

        cid = self.object_som.winner(x_s)
        if cid not in self.obj_model_map:
            self.obj_model_map[cid] = OnlineRegression(dimensions=self.feature_size+1)
        print "Picked regression model:", cid
        regressor = self.obj_model_map[cid]
        if len(regressor.Js) >= 1:
            old_mean = regressor.get_mean_err()
        else:
            old_mean = 0
        J = regressor.update(x_s, y_s)
        e_min_distance = self.effect_som.get_min_distance(y_s)
        print "Effect som min distance:", e_min_distance
        if e_min_distance == -1 or e_min_distance > self.epsilon_e:
            self.effect_som.add_neuron(y_s)
        else:
            self.effect_som.update(y_s)

        print "Effect som winner:", self.effect_som.winner(y_s)
        print "Before som #neurons:", self.object_som.x
        print "Effect som #neurons:", self.effect_som.x
        new_mean = regressor.get_mean_err()
        print "Mean J:", new_mean
        if J < self.epsilon_r and new_mean < old_mean:
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

    def __init__(self,feature_size=51, d0=0.2, sigma0=0.5, T1=100, T2=10):
        self.x = x # Num of columns
        self.y = y # Num of rows
        self.alpha0 = alpha0
        self.d0 = d0
        self.T1 = T1
        self.T2 = T2
        self.t = 0
        self.feature_size = feature_size

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

class OnlineRegression():

    def __init__(self, dimensions = 53, alpha0 = 0.2):
        self.name = 'online_regression'
        self.dimensions = dimensions
        self.alpha0 = alpha0
        self.W = np.random.rand(self.dimensions, self.dimensions)
        self.Js = []
        self.t = 0
        self.alpha_t = self.alpha0

    def get_mean_err(self):
        return np.mean(self.Js)

    def update(self, x, y):
        x_s = self.__preproc_x(x)
        y_s = y[np.newaxis].T
        y_s = np.vstack([y_s, [0.0]])
        J = self.get_square_error(x_s, y_s) / 2
        print J
        self.Js.append(J)
        dJdW = np.matmul(self.W, np.matmul(x_s, x_s.T)) - np.matmul(y_s, x_s.T)
        self.W -= self.alpha_t * dJdW
        alpha_t = self.alpha0*100/(self.t+100)
        self.t += 1
        return J

    def get_square_error(self, x, y):
        a = y - np.matmul(self.W, x)
        return np.matmul(a.T, a)[0][0]

    def predict(self, x):
        x_s = self.__preproc_x(x)
        return np.delete(np.matmul(self.W, x_s), self.dimensions-1, 0)

    def __preproc_x(self, x):
        x_s = x[np.newaxis].T
        x_s = np.vstack([x_s, [1.0]])
        return x_s

    def get_rmse(self, X, y):
        total = 0.0
        c = 0.0
        for i in range(len(X)):
            x = X[i]
            y_s = y[i][np.newaxis].T
            y_predicted = self.predict(x)
            a = y_s - y_predicted
            err = np.matmul(a.T, a)[0][0]
            total += err
            c += 1.0
        return math.sqrt(total / c)
