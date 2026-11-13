import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import Route, qr_decision

class TestRoute(unittest.TestCase):
    def test_add_and_limit(self):
        r = Route()
        for i in range(10): r.add(i, i)
        with self.assertRaises(ValueError): r.add(11, 11)
        self.assertEqual(len(r.points), 10)
    def test_minimum(self):
        r = Route()
        r.add(1, 2)
        self.assertFalse(r.ready())
        r.add(2, 2)
        self.assertTrue(r.ready())
    def test_loop(self):
        r = Route()
        r.add(0, 0); r.add(1, 1)
        self.assertEqual(r.advance(), (1, False))
        self.assertEqual(r.advance(), (0, True))
    def test_random_no_repeat(self):
        r = Route(mode='random')
        r.add(0, 0); r.add(1, 1); r.add(2, 2)
        for _ in range(30):
            old = r.index
            _, lap = r.advance()
            self.assertNotEqual(old, r.index)
            self.assertFalse(lap)
    def test_undo_clear(self):
        r = Route()
        r.add(0, 0); r.add(1, 1); r.undo()
        self.assertEqual(len(r.points), 1)
        r.clear(); self.assertEqual(r.index, 0)
    def test_qr(self):
        self.assertEqual(qr_decision(''), 'wait')
        self.assertEqual(qr_decision('PERSON_001'), 'granted')
        self.assertEqual(qr_decision('PERSON_999'), 'denied')

if __name__ == '__main__': unittest.main()
