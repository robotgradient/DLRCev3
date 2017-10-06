import os
from itertools import chain, cycle

from toolz import curry, pipe, partition_all

import numpy as np
from scipy import misc
from skimage import exposure

SUPPORTED_IMG_EXT = {".png"}


def is_image(path: str):
    return os.path.splitext(path)[1] in SUPPORTED_IMG_EXT


@curry
def use_dir(useable_dirs, path):
    return os.path.isdir(path) and os.path.basename(path) in useable_dirs


def img_to_array(path):
    """Reads image as a numpy array."""
    return np.array(misc.imread(path, mode="RGB"))


def img_paths(directory):
    return (os.path.join(root, f) for root, _, files in os.walk(directory) for f in files
            if is_image(f))


def select_images(rootdir, desired_subdirs):
    subs = (os.path.join(rootdir, sub) for sub in os.listdir(rootdir))
    desired = filter(use_dir(desired_subdirs), subs)
    return chain.from_iterable(map(img_paths, desired))


def rgb_equalize_hist(img: np.ndarray):
    return np.apply_along_axis(exposure.equalize_hist, 2, img)


def preproc_pipeline(img_path: str):
    """Convert an image path to a preprocessed image ready for training."""
    return pipe(img_path, img_to_array, lambda img: img / 255)


def keras_data_gen(dataroot, subdirs, batch_size):
    paths = select_images(dataroot, subdirs)
    for batch in cycle(partition_all(batch_size, paths)):
        yield (np.stack(list(map(preproc_pipeline, batch)), axis=0), None)


def rgb_z_score_norm(img: np.ndarray):
    """Normalizes one image array with a known mean and stdev."""
    mean = 127
    std_dev = 128
    return (img - mean) / std_dev


def read_z_normalized(img_path):
    """Combines z score normalization and reading image as an array."""
    return rgb_z_score_norm(img_array(img_path))
