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

    def nodeAdd(self, id,position):
        self.graphPoints_.insert(id,position)
    
    def nodeDelete(self,id):
        self.graphPoints_.pop(id)

    def nodeDistance(self,id1,id2) -> float:
        X = float(self.graphPoints_[id1][0] - self.graphPoints_[id2][0])
        Y = float(self.graphPoints_[id1][1] - self.graphPoints_[id2][1])

        return np.sqrt(np.power(X)+np.power(Y))

    def edgeAdd(self,parentId,childId):
        self.parents_.insert(childId,parentId)
    
    def edgeRemove(self,id):
        self.parents_.pop(id)

    def randomDirectionPoint(self)->int:
        x = int(random.uniform(0,self.winWidth_))
        y = int(random.uniform(0,self.winHight_))

        return x,y

    def nodeCollisionDetection(self)->bool:
        id = len(self.graphPoints_) - 1

        for rec in self.obstaclesList_:
            if rec.collidepoint(self.graphPoints_[id]):
                self.nodeDelete(id)
                return False
            
        return True
            
    def edgeCollisionDetection(self,nodeParent,nodeChild)->bool:
        
        for rec in self.obstaclesList_:
            for i in range(0,101):
                u = i/100
                x = nodeParent[0]*u + nodeChild[0]*(1-u)
                y = nodeParent[1]*u + nodeChild[1]*(1-u)

                if rec.collidepoint((x,y)):
                    return False
        return True

    def nodeConnection(self,parentId,childId)->bool:
        if self.edgeCollisionDetection(self.graphPoints_[parentId],self.graphPoints_[childId]):
            self.edgeAdd(parentId,childId)
            return True

        else:
            self.nodeDelete(childId)
            return False








    

        


def main():
    #map = Map((1000,600),(50,50),(510,510),30,100)
    rrt = RRT((1000,600),(50,50),(510,510),30,100)
    print(rrt.graphPoints_)
    rrt.drawScene()
    while(True):
        x,y = rrt.randomDirectionPoint()
        id = len(rrt.graphPoints_)
        rrt.nodeAdd(len(rrt.graphPoints_),(x,y))
        rrt.edgeAdd(id-1,id)
        if rrt.nodeCollisionDetection():
            pygame.draw.circle(rrt.WINDOW_,BLUE,rrt.graphPoints_[id],2,0)
            if rrt.edgeCollisionDetection(rrt.graphPoints_[id-1],rrt.graphPoints_[id]):
                pygame.draw.line(rrt.WINDOW_,PURPLE,rrt.graphPoints_[id-1],rrt.graphPoints_[id],2)
        pygame.display.update()
        
    #print(len(map.obstaclesList_))
    #print(map.obstNum_)
    
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    main()