from nn_object_detection.object_detectors import NNObjectDetector
from rick.async import AsyncCamera, AsyncObjectDetector


camera = AsyncCamera(0)
a_detector  = AsyncObjectDetector(camera)

while True:

	print(a_detector.closest_bbox)