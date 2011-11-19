
import genetics

class Critter(genetics.Individual):
    
    def __init__(self, morphology=None, neuralNet=None):
        self.morphology = morphology or self._createMorphology()
        self.neuralNet = neuralNet or self._createNeuralNet()
        
    def _createMorphology(self):
        return None
    
    def _createNeuralNet(self):
        return None
        