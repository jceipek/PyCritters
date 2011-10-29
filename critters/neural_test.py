
import unittest

from neural import InputNode, SumNode

class TestInputNode(unittest.TestCase):
    
    def setUp(self):
        self.node = InputNode()

    def test_process(self):
        # mirror input
        self.assertEquals(self.node.processReturn([2], 1), 2)
        
        # time independent
        self.assertEquals(self.node.processReturn([1], 1), 
                          self.node.processReturn([1], 2))

class TestSumNode(unittest.TestCase):
    
    def setUp(self):
        self.node = InputNode()

    def test_process(self):
        # mirror input
        self.assertEquals(self.node.processReturn([2], 1), 2)
        
        # time independent
        self.assertEquals(self.node.processReturn([1], 1), 
                          self.node.processReturn([1], 2))


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()