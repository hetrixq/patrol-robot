"""Offline hardware-independent regression: actual supplied QR images decode."""
import unittest
from pathlib import Path

try:
    import cv2
except ImportError:
    cv2 = None

BASE=Path(__file__).resolve().parents[1]/'tools'


@unittest.skipIf(cv2 is None, 'OpenCV missing')
class QRImageTests(unittest.TestCase):
    def test_allowed_qr(self):
        image=cv2.imread(str(BASE/'PERSON_001.png'))
        self.assertIsNotNone(image)
        code, corners, _=cv2.QRCodeDetector().detectAndDecode(image)
        self.assertEqual(code,'PERSON_001')
        self.assertIsNotNone(corners)

    def test_rejected_qr(self):
        image=cv2.imread(str(BASE/'PERSON_999.png'))
        self.assertIsNotNone(image)
        code, corners, _=cv2.QRCodeDetector().detectAndDecode(image)
        self.assertEqual(code,'PERSON_999')
        self.assertIsNotNone(corners)

if __name__=='__main__': unittest.main()
