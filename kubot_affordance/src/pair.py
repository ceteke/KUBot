import pickle

class Pair():

    def __init__(self, action, obj, model):
        self.action = action
        self.obj = obj
        self.model = model

    def save(self):
        pickle.dump(self.model,open('%s%s_%s_%s'%(self.action.base_path, self.action.name, self.obj.id, self.model.name), 'wb'))
