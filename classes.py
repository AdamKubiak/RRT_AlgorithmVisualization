import numpy as np
import math
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
        self.WINDOW_.fill(BLACK)
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
        self.isFinished_ = False
        self.graphPoints_ = []
        self.parents_ = []
        self.finalPosition_ = None
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

        return np.sqrt(np.power(X,2)+np.power(Y,2))

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

    def measureNodeDistance(self,nodeId):
        distanceMin = self.nodeDistance(0,nodeId)
        nearestId = 0
        for i in range(0,nodeId):
            if self.nodeDistance(i,nodeId) < distanceMin:
                nearestId = i
                distanceMin = self.nodeDistance(i,nodeId)
        return nearestId

    #matma ;_;
    def stepMove(self,nodeNearestId,nodeRandomId,stepSize = 7)->None:
        distance = self.nodeDistance(nodeNearestId,nodeRandomId)

        if distance > stepSize:
            u = stepSize/distance

            nodeNearestPosition = (self.graphPoints_[nodeNearestId][0],self.graphPoints_[nodeNearestId][1])
            nodeRandomPosition = (self.graphPoints_[nodeRandomId][0],self.graphPoints_[nodeRandomId][1])

            diffX = nodeRandomPosition[0] - nodeNearestPosition[0]
            diffY = nodeRandomPosition[1] - nodeNearestPosition[1]

            theta = math.atan2(diffY,diffX)

            nodePosition = (int(nodeNearestPosition[0] + stepSize*math.cos(theta)),
                            int(nodeNearestPosition[1]+stepSize*math.sin(theta)))
            self.nodeDelete(nodeRandomId)

            if abs(nodePosition[0] - self.endPos_[0])<stepSize and abs(nodeNearestPosition[1] - self.endPos_[1]) <= 20:
                self.nodeAdd(nodeRandomId,self.endPos_)
                self.finalPosition_ = nodeRandomId
                self.isFinished_ = True
            
            else:
                self.nodeAdd(nodeRandomId,nodePosition)
    
    def moveToEndPos(self,nodeEndPosition):
        tempId = len(self.graphPoints_)

        self.nodeAdd(tempId,nodeEndPosition)

        nearestToEndPos = self.measureNodeDistance(tempId)
        self.stepMove(nearestToEndPos,tempId)
        self.nodeConnection(nearestToEndPos,tempId)
    
    def expandTree(self):
        tempId = len(self.graphPoints_)

        randomNodePosition = self.randomDirectionPoint()
        self.nodeAdd(tempId,randomNodePosition)

        if self.nodeCollisionDetection():
            nearestNode = self.measureNodeDistance(tempId)
            self.stepMove(nearestNode,tempId)
            self.nodeConnection(nearestNode,tempId)
    
    def drawFinalPath(self):
        positionId = len(self.graphPoints_) -1
        while positionId >=0:
            pygame.draw.circle(self.WINDOW_,RED,self.graphPoints_[positionId],2,0)
            
            tempPosition = positionId
            positionId = self.parents_[positionId]
            pygame.draw.line(self.WINDOW_,RED,self.graphPoints_[tempPosition],self.graphPoints_[positionId],2)
            pygame.display.update()
        
        return True




def main():
    rrt = RRT((1000,600),(50,50),(510,510),30,200)
    
    rrt.drawScene()
    iteration = 0     
        
    while (rrt.isFinished_ == False):
        #id = len(rrt.graphPoints_)
        if iteration % 5 == 0 and rrt.isFinished_ == False:
            rrt.moveToEndPos(rrt.endPos_)
            pygame.draw.circle(rrt.WINDOW_,BLUE,rrt.graphPoints_[-1],2,0)
            pygame.draw.line(rrt.WINDOW_,PURPLE,rrt.graphPoints_[-1],rrt.graphPoints_[rrt.parents_[-1]],2)
            pygame.display.update()

        else:
            rrt.expandTree()
            pygame.draw.circle(rrt.WINDOW_,BLUE,rrt.graphPoints_[-1],2,0)
            pygame.draw.line(rrt.WINDOW_,PURPLE,rrt.graphPoints_[-1],rrt.graphPoints_[rrt.parents_[-1]],2)

        iteration+=1

        pygame.display.update()

        
    rrt.drawFinalPath()
    print(len(rrt.graphPoints_))
    print(len(rrt.parents_))

    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    main()