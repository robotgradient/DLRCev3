"""Creates a bunch of images with a plain background."""

from itertools import product
from pathlib import Path
from functools import partial
import os

from PIL import Image

DATA_DIR = Path.home() / "dlrc/data/colors"
os.makedirs(DATA_DIR, exist_ok=True)

# step of 10 generates 17,575 images!
STEP = 10

def color_range(step):
    return range(0, 255, step)

color_combos = product(color_range(STEP), color_range(STEP), color_range(STEP))
for num_id, (red, green, blue) in enumerate(color_combos):
    im = Image.new(mode='RGB', size=(64, 64), color=(red, green, blue))
    im.save(DATA_DIR / "plain-{}.png".format(num_id))
