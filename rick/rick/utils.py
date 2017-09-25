import numpy as np

def bbox_center(xmin, ymin, xmax, ymax):
	return np.asarray([(xmin + xmax)/2., (ymin + ymax)/2.])


def bbox_bottom_center(xmin, ymin, xmax, ymax):
		return np.asarray([(xmin + xmax)/2., ymax])


