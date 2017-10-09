import pygame
import numpy as np


class RemoteDisplay:
    """Creates remotestuff"""

    def __init__(self):
        pygame.init()

        pygame.display.set_caption('Video Feed')
        self.screen = pygame.display.set_mode((480, 640))

    def imshow(self, name, thing):
        bv = self.screen.get_view("0")
        bv.write(thing.tostring())
        pygame.display.flip()

    def waitKey(self, n):
        # -1 means no key was pressed
        return -1

    def destroyAllWindows(self):
        pygame.display.quit()