import os
from itertools import cycle

import cv2
import numpy as np


def target_input_from_directory(directory):
    return [os.path.join(root, "target.png") for root, _, files in os.walk(directory)
            for _ in files]


def input_from_directory(directory):
    return [os.path.join(root, f) for root, _, files in os.walk(directory)
            for f in files if f != "target.png"]


def chunk(file_iter, batch_size):
    files = list(file_iter)
    return (files[i:i + batch_size] for i in range(0, len(files), batch_size))


def blender_data_gen(directory, batch_size):
    inputs = chunk(input_from_directory(directory), batch_size)
    for batch in cycle(inputs):
        yield np.stack(list(map(cv2.imread, batch)), axis=0)


if __name__ == '__main__':
    endless = blender_data_gen("/home/quickbeam/Downloads/Img/train", 4)
    counter = 0
    while counter < 10:
        print(next(endless))
        counter += 1
