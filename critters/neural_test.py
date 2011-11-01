
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

    def test_process(self):
        self.node = neural.SumThresholdNode(threshold=1)
        
        nums = [1,-1,2,4,3.2, 4.1]
        self.assertEquals(self.node._processReturn(nums, 1), 1)
        
        nums = [1,-1,2,-4,-3.2, 4.1]
        self.assertEquals(self.node._processReturn(nums, 1), 0)
        
        nums = [1]
        self.assertEquals(self.node._processReturn(nums, 1), 1)
        
        numSets = [[1,-1,2,4,3.2, 4.1], [1], [1,-2]]
        for nums in numSets:
            self.assertTimeIndependent(nums)
    
    def test_diffThreshold(self):
        self.node = neural.SumThresholdNode(threshold=0)
        
        nums = [1,-1,2,4,3.2, 4.1]
        self.assertEquals(self.node._processReturn(nums, 1), 1)
        
        nums = [1,-1,2,-4,-3.2, 4.1]
        self.assertEquals(self.node._processReturn(nums, 1), 0)
        
        nums = [0]
        self.assertEquals(self.node._processReturn(nums, 1), 1)
        
        numSets = [[1,-1,2,4,3.2, 4.1], [0], [1,-2]]
        for nums in numSets:
            self.assertTimeIndependent(nums)
            
    def test_randomThreshold(self):
        for _ in range(100):
            node = neural.SumThresholdNode()
            self.assertTrue(0.0 <= node.threshold and node.threshold <= 1.0)

class TestGreaterThanNode(NodeTest):
    
    def setUp(self): self.node = neural.GreaterThanNode()

    def test_process(self):
        numSets = [[-1,-0.5,0,4], [1], [3, 2], [4, -4, 4], [1,2,3,2,4]]
        for nums in numSets:
            expected = int(nums == sorted(nums))
            self.assertEquals(self.node._processReturn(nums, 1), expected)
            self.assertTimeIndependent(nums)
        
class TestSignOfNode(NodeTest):
    
    def setUp(self): self.node = neural.SignOfNode()

    def test_process(self):
        nums = [-5, -0.5, 0, 0.5, 1, 3]
        for x in nums:
            self.assertEquals(self.node._processReturn([x], 1), utils.sign(x))
            self.assertTimeIndependent([x])

class TestMinNode(NodeTest):
    
    def setUp(self): self.node = neural.MinNode()

    def test_process(self):
        numSets = [[-1,-0.5,0,4], [1], [3, 2], [4, -4, 4], [1,2,3,2,4]]
        for nums in numSets:
            self.assertEquals(self.node._processReturn(nums, 1), min(nums))
            self.assertTimeIndependent(nums)
            
class TestMaxNode(NodeTest):
    
    def setUp(self): self.node = neural.MaxNode()

    def test_process(self):
        numSets = [[-1,-0.5,0,4], [1], [3, 2], [4, -4, 4], [1,2,3,2,4]]
        for nums in numSets:
            self.assertEquals(self.node._processReturn(nums, 1), max(nums))
            self.assertTimeIndependent(nums)
            
class TestAbsNode(NodeTest):
    
    def setUp(self): self.node = neural.AbsNode()

    def test_process(self):
        nums = [-5, -0.5, 0, 0.5, 1, 3]
        for x in nums:
            self.assertEquals(self.node._processReturn([x], 1), abs(x))
            self.assertTimeIndependent([x])

class TestIfNode(NodeTest):
    
    def setUp(self): self.node = neural.IfNode()

    def test_process(self):
        nums = [0, 0, -1]
        self.assertEquals(self.node._processReturn(nums, 1), 0)
        
        nums = [2, 3, 4]
        self.assertEquals(self.node._processReturn(nums, 1), 2)
        
        nums = [-1, 3, 4]
        self.assertEquals(self.node._processReturn(nums, 1), 3)
        
        nums = [0, 0, 0]
        self.assertEquals(self.node._processReturn(nums, 1), 0)
        
        numSets = [[0, 0, -1], [2, 3, 4], [-1, 3, 4], [0, 0, 0]]
        for nums in numSets:
            self.assertTimeIndependent(nums)

class TestPackageLevel(unittest.TestCase):
    
    def test_nodeTypes(self):
        def assertNodeSubclass(cls):
            self.assertTrue(issubclass(cls, neural.Node))
            self.assertIsNot(cls, neural.Node)
        
        includingInput = neural.nodeTypes(includeInput=True)
        for t in includingInput: assertNodeSubclass(t)
        self.assertTrue(neural.InputNode in includingInput)
        
        notIncludingInput = neural.nodeTypes(includeInput=False)
        for t in notIncludingInput: assertNodeSubclass(t)
        self.assertTrue(neural.InputNode not in notIncludingInput)
        
    def test_randomNodes(self):
        for size in [1, 2, 10, 100]:
            nodes = neural.randomNodes(size)
            self.assertEquals(len(nodes), size)
            self.assertTrue(all(isinstance(node, neural.Node) 
                                for node in nodes))
        
        

if __name__ == "__main__":
    unittest.main()
