###############################################
# ITECH 2020 Computational Design Thinking Final Assignment
# By Guillaume, Simon, Tim, Gabriel
###############################################
# 3D Printing Agent-Based BirdFlocking
###############################################

import Rhino.Geometry as rg
import random as rnd

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
            bird.ComputeNestToFoodVector()
            bird.ComputeFlockingVector()
            bird.ComputeWindVector()
        for bird in self.Birds:
            bird.Update()


class PredatorSystem:
    def __init__():
        pass

class Agent:
    def __init__(self, initialPosition):
        self.Position = initialPosition
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)

    def ComputeAvoidObstacleVector(self):
        pass

    def ComputeWindVector(self):
        self.DesiredVelocity += rg.Vector3d(iWindSpeedX,0.0,0.0)

    def ComputeUpliftVector(self):
        uplift = EnvironmentBoundary.CreateUpliftArea()
        #if self.Position.X ==  
        #rg.Cylinder.


class Bird(Agent):
    def __init__(self, initialPosition):
        self.Position = initialPosition
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)
        self.History = [self.Position]

    def ComputeFoodToNestVector(self):
        pass

    def ComputeNestToFoodVector(self):
        toFood = iFoodSource - self.Position
        toFood.Unitize()
        toFood *= iFoodStrength**2
        self.DesiredVelocity = toFood

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
            
   
            #print(_bird)
    def ComputeAvoidPredatorVector(self):
        pass

    def Die(self):
        pass

    def Update(self):
        self.Velocity = 0.97 * self.Velocity + 0.03 * self.DesiredVelocity
        if (self.Velocity.Length > 0.1):
            self.Velocity.Unitize()
            self.Velocity *= iSpeed
            self.Position += self.Velocity
            self.History.append(self.Position)

class Predator(Agent):
    def __init__():
        pass

    #Defining Vectors

    def ComputeBirdDensityVector(self):
        pass

    def ComputeRandomMovement(self):
        pass

    def EatBird(self):
        pass

    #Updating Final Vector
    def Update(self):
        pass


#Initialize BirdSystem 
if iEnabled == True:
    if iReset:
        birdSystem = BirdSystem(iAgentCount)
    else: 
        birdSystem.Update()

birdHistory = []
birdPositions = []

for bird in birdSystem.Birds:
    birdPositions.append(bird.Position)
    birdHistory.append(rg.PolylineCurve(bird.History))


oHistory = birdHistory
oPositions = birdPositions