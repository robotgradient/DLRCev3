import os
from scipy import misc as m
import sys

for root, _, files in os.walk(sys.argv[1]):
    for im_path in files:
        im_path = os.path.join(root, im_path)
        im = m.imread(im_path)
        im = m.imresize(im, size=(64, 64))
        m.imsave(im_path, im)
