
import unittest
import utils


class TestUtils(unittest.TestCase):

    def test_sign(self):
        self.assertEquals(utils.sign(-5.3), -1)
        self.assertEquals(utils.sign(-0.5), -1)
        self.assertEquals(utils.sign(-0.0000001), -1)
        self.assertEquals(utils.sign(0), 0)
        self.assertEquals(utils.sign(0.00000001), 1)
        self.assertEquals(utils.sign(0.3), 1)
        self.assertEquals(utils.sign(324), 1)
        
    def test_product(self):
        self.assertEquals(utils.product([]), 1)
        self.assertAlmostEquals(utils.product([2.1]), 2.1)
        self.assertAlmostEquals(utils.product([2.1, 4.3, 2.1]), 18.963)
        self.assertAlmostEquals(utils.product([-0.5, 10.1, 3]), 
                                -15.149999999999999)
        
    def test_divide(self):
        self.assertEquals(utils.divide([]), 1)
        self.assertAlmostEquals(utils.divide([2.1]), 0.47619047619047616)
        self.assertAlmostEquals(utils.divide([2.1, 4.3, 2.1]), 
                                0.052734272003374986)
        self.assertAlmostEquals(utils.divide([-0.5, 10.1, 3]), 
                                -0.06600660066006601)


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()