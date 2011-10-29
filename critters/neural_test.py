
import unittest
import neural
import utils

class NodeTest(unittest.TestCase):
    
    def assertTimeIndependent(self, inputs):
        self.assertEquals(self.node._processReturn(inputs, 1), 
                          self.node._processReturn(inputs, 2))
        
        self.assertEquals(self.node._processReturn(inputs, 0.3), 
                          self.node._processReturn(inputs, 0.2))

class TestInputNode(NodeTest):
    
    def setUp(self): self.node = neural.InputNode()

    def test_process(self):
        # mirror input
        self.assertEquals(self.node._processReturn([2], 1), 2)
        self.assertTimeIndependent([0.5])

class TestSumNode(NodeTest):
    
    def setUp(self): self.node = neural.SumNode()

    def test_process(self):
        numSets = [[1,-1,2,4,3.2, 4.1], [1], [0], [-1,2]]
        for nums in numSets:
            self.assertEquals(self.node._processReturn(nums, 1), sum(nums))
            self.assertTimeIndependent(nums)

class TestProductNode(NodeTest):
    
    def setUp(self): self.node = neural.ProductNode()

    def test_process(self):
        numSets = [[1,-1,2,4,3.2, 4.1], [1], [0], [-1,2]]
        for nums in numSets:
            self.assertAlmostEquals(self.node._processReturn(nums, 1), 
                                    utils.product(nums))
            self.assertTimeIndependent(nums)

class TestDivideNode(NodeTest):
    
    def setUp(self): self.node = neural.DivideNode()

    def test_process(self):
        numSets = [[1,-1,2,4,3.2, 4.1], [1], [-1,2]]
        for nums in numSets:
            self.assertAlmostEquals(self.node._processReturn(nums, 1), 
                                    utils.divide(nums))
            self.assertTimeIndependent(nums)
            
class TestSumThresholdNode(NodeTest):
    
    def setUp(self): self.node = neural.SumThresholdNode()

    def test_process(self):
        numSets = [[1,-1,2,4,3.2, 4.1], [1], [-1,2]]
        for nums in numSets:
            self.assertAlmostEquals(self.node._processReturn(nums, 1), 
                                    utils.divide(nums))
            self.assertTimeIndependent(nums)

if __name__ == "__main__":
    unittest.main()