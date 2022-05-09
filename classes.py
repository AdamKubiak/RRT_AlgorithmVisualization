from turtle import window_height
import numpy as np
import random
import pygame
import colorspy as color
import pygments

RED = color.red
GREEN = color.green
BLUE = color.blue
YELLOW = color.yellow
WHITE = color.white
BLACK = color.black
PURPLE = color.purple
ORANGE = color.orange
GREY = color.gray
TURQUOISE = color.turquoise


class Map:
    def __init__(self, windowSize, startPos, endPos, obstSize,obstNum) -> None:
        self.winWidth_ = windowSize[0]
        self.winHight_ = windowSize[1]
        self.startPos_ = startPos
        self.endPos_ = endPos
        self.obstSize_ = obstSize
        self.obstNum_ = obstNum

        self.obstaclesList_ = []
        
        pygame.init()
        self.WINDOW_ = pygame.display.set_mode(windowSize)
        self.WINDOW_.fill(WHITE)
        pygame.display.set_caption('RRT Vis')
    #generowanie współrzędnych, dodanie do listy i rysowanie przeszkody
    def createRandomObstacles(self) -> None:

        for i in range(self.obstNum_):
            x = int(random.uniform(0,self.winWidth_- self.obstSize_))
            y = int(random.uniform(0,self.winHight_ - self.obstSize_))

            rect = pygame.Rect((x,y),(self.obstSize_,self.obstSize_))

            if rect.collidepoint(self.startPos_) or rect.collidepoint(self.endPos_):
                pass
            else:
                self.obstaclesList_.append(rect)

        self.obstNum_ = len(self.obstaclesList_)
    
    def drawObstacles(self) -> None:
        self.createRandomObstacles()
        for rect in self.obstaclesList_:
            pygame.draw.rect(self.WINDOW_,GREEN,rect)

    def drawScene(self):
        pygame.draw.circle(self.WINDOW_,RED,self.startPos_,5,0)
        pygame.draw.circle(self.WINDOW_,RED,self.endPos_,20,1)
        self.drawObstacles()

class RRT(Map):
    def __init__(self,windowSize, startPos, endPos, obstSize,obstNum) -> None:
        Map.__init__(self,windowSize, startPos, endPos, obstSize,obstNum)
        self.goalFlag = False
        self.graphPoints_ = []
        self.parents_ = []
        self.goalstate = None
        self.path = []

        self.graphPoints_.append(self.startPos_)
        self.parents_.append(0)
        
        


def main():
    #map = Map((1000,600),(50,50),(510,510),30,100)
    rrt = RRT((1000,600),(50,50),(510,510),30,100)
    print(rrt.graphPoints_)
    rrt.drawScene()
    #print(len(map.obstaclesList_))
    #print(map.obstNum_)
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    main()