"""Functions for working with blender-generated images."""
import os
from itertools import cycle

import numpy as np
from scipy import misc


def img_array(path):
    return np.array(misc.imread(path, mode="RGB"))


def rgb_z_score_norm(img):
    """Normalizes one image array with a known mean and stdev."""
    mean = 127
    std_dev = 128
    return (img - mean) / std_dev


def read_normalized(img_path):
    return rgb_z_score_norm(img_array(img_path))


def img_paths(directory):
    return [os.path.join(root, f) for root, _, files in os.walk(directory)
            for f in files if f != "target.png"]


def chunked(file_iter, batch_size):
    """Splits an iterator into chunks.

    Currently needs to turn the iterator into a list, so be careful about memory.
    """
    files = list(file_iter)
    return (files[i:i + batch_size] for i in range(0, len(files), batch_size))


def blender_data_gen(directory, batch_size):
    """Feeds data in a way that plays well with Keras's `fit_generator`"""
    paths = chunked(img_paths(directory), batch_size)
    for batch in cycle(paths):
        arrays = map(read_normalized, batch)
        yield (np.stack(list(arrays), axis=0),None)
