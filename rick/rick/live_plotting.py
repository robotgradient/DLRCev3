import pygame
from multiprocessing import Process, Pipe
import threading
import os
from collections import deque

def draw_brick(x,y, screen, color=(0, 128, 255)):

    x=int(x/300 * 600+ 100)
    y=int(y/300 * 600+ 100)
    size = 5
    pygame.draw.rect(screen, color, pygame.Rect(x-size, y-size, 2*size, 2*size))

def draw_ppoint(x,y, screen, color=(0, 128, 30)):

    x=int(x/300 * 600 + 100)
    y=int(y/300 * 600 + 100)
    size = 2
    pygame.draw.rect(screen, color, pygame.Rect(x-size, y-size, 2*size, 2*size))

class MapRenderer():

    def __init__(self, window_size=(600, 600), map_size=(300.,300.)):
        super(MapRenderer, self).__init__()
        self.map_size = map_size
        self.window_size = window_size
        pygame.init()
        pygame.display.set_caption('Map')
        self.screen = pygame.display.set_mode(window_size)
        self.robot_path = deque(maxlen=400)


    def plot_bricks(self, bricks):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        for brick in bricks:
            draw_brick(brick[0], brick[1], self.screen)
        pygame.display.flip()

    def plot_robot(self, x,y):
        # self.robot_path.append([x,y])
        # for coords in self.robot_path:
        #     draw_ppoint(coords[0], coords[1], self.screen)
        draw_brick(x,y,self.screen, (255,0,0))

    def plot_target_brick(self, x, y):
        draw_brick(x, y, self.screen, (100, 50, 200))


    def plot_bricks_and_trajectory(self, bricks, trajectory):

        self.screen.fill((0,0,0))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        for coords in trajectory:
            draw_ppoint(coords[0], coords[1], self.screen)
        for brick in bricks:
            color = (0, 0, 250) if brick[-1] != 5 else (250,0,0)
            draw_brick(brick[0], brick[1], self.screen, color)
        pygame.display.flip()



    def refresh(self):
        self.screen.fill((0,0,0))


if __name__=="__main__":

    bricks = [[100.4, 100.4], [20, 20], [250, 100]]
    trajectory = [[30,3], [31,3], [32,3], [33,3], [34,3]]
    robot = [50,100]
    map = MapRenderer()

    while True:
        robot[0]+=0.4
        map.plot_bricks_and_trajectory(bricks,trajectory)