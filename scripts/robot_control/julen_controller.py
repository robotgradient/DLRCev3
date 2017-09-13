
import numpy as np


def wheelvelcontroller(relObjPos, K1,K2 = 0):


	# Move to circular params

	relObjCir[0] = np.sqrt(np.pow(relObjPos(1),2) + np.pow(relObjPos(2),2));
	relObjCir[1] = np.atan(relObjPos(1)/relObjPos(2))

	# From circular params to wheels vel

	rotationMat = np.matrix([K1/2,K1/2],[K2,K2])

	velWheels = np.matmul(rotationMat,relObjCir)

	return velWheels


def ImageDot2relPos(pixelVect):


	FoV = 75
	Height = 10

	alfa = 30

	# IMAGE = 640 x 480 : 
	Npx = 640
	Npy = 480

	FoVpx = FoV/Npx
	FoVpy = FoV/Npy

	x2 = Height*np.tan()


	


























