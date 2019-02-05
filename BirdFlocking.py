###############################################
# ITECH 2020 Computational Design Thinking Final Assignment
# By Guillaume, Simon, Tim, Gabriel
###############################################
# 3D Printing Agent-Based BirdFlocking
###############################################


#Todo : 
#Multiple birds flocks
#Predator needs to be able to eat the birds
#Birds should have the behavior to get back to their nests
#Predator should be able to idle after eating a bird (When not hungry anymore)
#Set up the environment properties (Boundaries & obstacles)
#Set up a wind map


import Rhino.Geometry as rg
import random as rnd
import math

class EnvironmentBoundary:
    def __init__():
        pass

    def CreateBoundaryBox(self):
        pass

    def CreateObstacle3d(self):
        pass

    def CreateUpliftArea(self):
        pass

    def CreateWindGradient(self):
        pass


class BirdSystem:
    #Creating different types of agents
    #Realtime flocking optimization using R-Tree system.
    def __init__(self,agentCount):
        self.Birds = []
        for i in range(0,agentCount):
            self.Birds.append(Bird(rg.Point3d(rnd.uniform(0.0,50.0),rnd.uniform(0.0,50.0),rnd.uniform(0.0,50.0))))

    def Update(self):
        for bird in self.Birds:
            bird.ComputeRandomMovement(0.05)
            bird.ComputeNestToFoodVector()
            bird.ComputeFlockingVector()
            bird.ComputeWindVector(30)
            #bird.ComputeAvoidObstacleVector() #Not used!!!
            bird.ComputeAvoidBrepsVector(50)
            #bird.ComputeUpliftVector()  #Not used!!!
            bird.ComputeUpliftRevolveVector(1.0, 3.0, 4.0)
            bird.ComputeAvoidPredatorVector(10)
        for bird in self.Birds:
            bird.Update()


class PredatorSystem:
    def __init__(self, predatorCount):
        self.Predators = []
        for i in range(0,predatorCount):
            self.Predators.append(Predator(rg.Point3d(rnd.uniform(0.0,50.0),rnd.uniform(0.0,50.0),rnd.uniform(0.0,50.0))))

    def Update(self):
        for predator in self.Predators:
            predator.ComputeRandomMovement(0.2)
            predator.SearchClosestBird(birdPositions)
            predator.WaitingForFood(2.0, 0.4, 0.5, 0.01)
            predator.ComputeAvoidBrepsVector(25)
            predator.ComputeUpliftRevolveVector(0.05, 0.4, 0.5)
            predator.ComputeWindVector(0.1)
        for predator in self.Predators:
            predator.Update()


class Agent:
    def __init__(self, initialPosition):
        self.Position = initialPosition
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)

    def ComputeRandomMovement(self, value):
        randomVec = rg.Vector3d(rnd.uniform(-value, value),rnd.uniform(-value, value),rnd.uniform(-value, value))
        self.DesiredVelocity = randomVec

    def ComputeAvoidObstacleVector(self, strength): #Not used!!!!
        ObjectCollide = iObstacle - self.Position
        ObjectDistance = ObjectCollide.Length     
        if ObjectDistance < (iObstacleSize + iDetectonDistance):
            ObjectCollide.Unitize()
            ObjectCollide *= -(1- (ObjectDistance / iDetectonDistance))**2*10*iCollideStrength
            ObjectCollide.Unitize()
            self.DesiredVelocity += (ObjectCollide*strength)


    def ComputeAvoidBrepsVector(self, strength):
        for obstacle in iObstacleBreps:
            projectedPt = obstacle.ClosestPoint(self.Position)
            ObjectCollide = projectedPt - self.Position
            ObjectDistance = ObjectCollide.Length
            if ObjectDistance < iDetectonDistance:
                ObjectCollide.Unitize()
                ObjectCollide *= -(1- (ObjectDistance / iDetectonDistance))**2 #better formula?
                ObjectCollide.Unitize()
                self.DesiredVelocity += (ObjectCollide*strength)


    def ComputeWindVector(self, strength):
        iWindSpeed.Unitize()
        self.DesiredVelocity += (iWindSpeed*strength)

    def ComputeUpliftVector(self, strength): #Not used!!!!
        for i in range(len(iUpliftArea)):
            testInside = iUpliftArea[i].IsPointInside(self.Position, 0.1, True)
            if testInside == True:
                upliftVector = rg.Vector3d(0.0, 0.0, iUpliftStrength[i]) 
                upliftVecotr.Unitize()
                self.DesiredVelocity += (upliftVector*strength)

    def ComputeUpliftRevolveVector(self, strength, valueMin, valueMax):
        for i in range(len(iUpliftCrvs)):
            t = iUpliftCrvs[i].ClosestPoint(self.Position)[1]
            projectedPt = iUpliftCrvs[i].PointAt(t)
            vec = projectedPt - self.Position
            length = vec.Length
            if length < iUpliftRadius[i]:
                vector = (rg.Vector3d(0.0, 0.0, iUpliftStrength[i]*200) / (length*length/10+5))
                if self.Rotation == True:
                    vector.Rotate(rnd.uniform(valueMin, valueMax), rg.Vector3d(0,0,1))
                else: 
                    vector.Rotate(rnd.uniform(-valueMin, -valueMax), rg.Vector3d(0,0,1))
                vector.Unitize()
                self.DesiredVelocity += (vector*strength)


            

class Bird(Agent):
    def __init__(self, initialPosition):
        self.Position = initialPosition
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)
        self.History = [self.Position]
        if rnd.choice([0, 1]) > 0.5:
            self.Rotation = True
        else: 
            self.Rotation = False


    def ComputeFoodToNestVector(self):
        pass

    def ComputeNestToFoodVector(self):
        toFood = iFoodSource - self.Position
        toFood.Unitize()
        toFood *= iFoodStrength**2
        self.DesiredVelocity += toFood

    def ComputeFlockingVector(self):
        #Separation

        #Alignment

        #Cohesion
        #Try to find birds of same colour and attract to them up to a certain seperation distance.
        #If to close -> get away until seperationdistance = true
        

        _birdPositions = []
        for bird in birdSystem.Birds:
            _birdPositions.append(bird.Position)

        ClosestBirds = list(rg.RTree.Point3dClosestPoints(_birdPositions,[self.Position],iDetectonDistance))

        for closebird in ClosestBirds[0]:
            if _birdPositions[closebird] == self.Position: continue
            goToBird = _birdPositions[closebird] - self.Position
            distance = goToBird.Length
                       
            if (distance < iSeparationDistance):
                getAway = self.Position - _birdPositions[closebird]
                getAwayDistance = getAway.Length
                getAway.Unitize()
                getAway *= iSeparationStrength * (1 - (getAwayDistance/iSeparationDistance))**2
                self.DesiredVelocity += getAway

            else:
                goToBird.Unitize()
                goToBird *= (iSeparationStrength / (distance + 1))**2
                self.DesiredVelocity += goToBird
            
    def ComputeAvoidPredatorVector(self, strength):
        for predator in predatorSystem.Predators:
            getAway = self.Position - predator.Position 
            distance = getAway.Length
            if distance < iDetectonDistance:
                getAway.Unitize()
                self.DesiredVelocity += (getAway*strength)

    def Die(self):
        pass

    def Update(self):
        self.Velocity = (1-iAgentVelocity) * self.Velocity + iAgentVelocity * self.DesiredVelocity
        if (self.Velocity.Length > 0.1):
            self.Velocity.Unitize()
            self.Velocity *= iSpeed
            self.Position += self.Velocity
            self.History.append(self.Position)

class Predator(Agent):
    def __init__(self, initialPosition):
        self.Position = initialPosition
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)
        self.History = [self.Position]
        self.Hunting = False
        #True is positive, false is negative rotation
        if rnd.choice([0, 1]) > 0.5:
            self.Rotation = True
        else: 
            self.Rotation = False
    #Defining Vectors

    def SearchClosestBird(self, birds):
        position = self.Position
        closestBird = self.Position
        distance = iPredatorRadius
        for bird in birds:
            dis = position.DistanceTo(bird)
            if dis < distance:
                closestBird = bird
                distance = dis
        if distance < iPredatorRadius:
            targetVec = closestBird - self.Position
            targetVec.Unitize()
            self.DesiredVelocity += (targetVec * iPredatorSpeed)
            self.Hunting = True
        else:
            self.Hunting = False

    def WaitingForFood(self, strength, valueMin, valueMax, upliftStrength):
        if self.Hunting == False: 
            vec = rg.Vector3d(self.Velocity.X, self.Velocity.Y, 0.0)
            if self.Rotation == True:
                vec.Rotate(rnd.uniform(valueMin, valueMax), rg.Vector3d(0,0,1))
            else: 
                vec.Rotate(rnd.uniform(-valueMin, -valueMax), rg.Vector3d(0,0,1))
            vec += rg.Vector3d(0.0, 0.0, upliftStrength)
            vec.Unitize()
            self.DesiredVelocity += (vec*strength)

    def ComputeBirdDensityVector(self):
        pass

    def EatBird(self):
        pass

    def Update(self):
        self.Velocity = (1-iPredatorVelocity) * self.Velocity + iPredatorVelocity * self.DesiredVelocity
        if (self.Velocity.Length > 0.1):
            self.Velocity.Unitize()
            self.Velocity *= iSpeed
            self.Position += self.Velocity
            self.History.append(self.Position)


#Initialize BirdSystem & PredatorSystem
if iEnabled == True or iReset:
    if iReset or "birdSystem" not in globals():
        birdSystem = BirdSystem(iAgentCount)
        predatorSystem = PredatorSystem(iPredatorCount)
    else: 
        birdSystem.Update()
        predatorSystem.Update()

birdHistory = []
birdPositions = []
predatorHistory = []
predatorPositions = []

for bird in birdSystem.Birds:
    birdPositions.append(bird.Position)
    birdHistory.append(rg.PolylineCurve(bird.History))

for predator in predatorSystem.Predators:
    predatorPositions.append(predator.Position)
    predatorHistory.append(rg.PolylineCurve(predator.History))

oBirdHistory = birdHistory
oBirdPositions = birdPositions

oPredatorHistory = predatorHistory
oPredatorPositions = predatorPositions
