###############################################
# ITECH 2020 Computational Design Thinking Final Assignment
# By Guillaume, Simon, Tim, Gabriel
###############################################
# 3D Printing Agent-Based BirdFlocking
###############################################

import Rhino.Geometry as rg
import random as rnd
import math
import copy

class EnvironmentSystem:
    def __init__(self):
        self.Wind = rg.Vector3d(rnd.uniform(-1.0, 1.0), rnd.uniform(-1.0, 1.0), 0.0)
        self.WindSpeed = iWindSpeed
        self.Wind.Unitize()
        self.Wind *= self.WindSpeed
        self.Food = "Not initialized"


    def SpawnFood(self, amountSpawns, respawnTime, initFood):
        if self.Food == "Not initialized":
            self.Food = iFoodSpawnLocations
            amountOfFood = []
            for i in range(amountSpawns):
                amountOfFood.append(initFood)
            for i in range(len(self.Food)-amountSpawns):
                amountOfFood.append(0)
            rnd.shuffle(amountOfFood)
            self.AmountFood = amountOfFood
        else:
            existing = 0
            possibleLocations = []
            for i in range(len(self.AmountFood)):
                if self.AmountFood[i] > 0:
                    existing += 1
                elif self.AmountFood[i] <= (-respawnTime):
                    possibleLocations.append(i)
            if existing < amountSpawns and len(possibleLocations) > 0:          
                newLocation = rnd.choice(possibleLocations)
                self.AmountFood[newLocation] = initFood

    def InitializeFood(self, amount): 
        if self.AmountFood == "Not initialized":
            self.AmountFood = []
            for food in self.Food:
                self.AmountFood.append(amount)
        for i in range(len(self.AmountFood)):
            if self.AmountFood[i] <= 0:
                del self.AmountFood[i]
                del self.Food[i]
    

    #Computes the amount of food left and despawns it if nothing is left
    def ComputeFoodLive(self, eatDistance):
        if "birdSystem" in globals():
            _birdPositions = []
            for bird in birdSystem.Birds:
                _birdPositions.append(bird.Position)
            for i in range(len(self.Food)):
                if self.AmountFood[i] <= 0:
                    self.AmountFood[i] -= 1 
                else: 
                    closestBirds = list(rg.RTree.Point3dClosestPoints(_birdPositions,[self.Food[i]], eatDistance))
                    self.AmountFood[i] -= len(closestBirds[0])
 


    def CreateBoundaryBox(self):
        pass

    def CreateObstacle3d(self):
        pass

    def CreateUpliftArea(self):
        pass

    def CreateWindGradient(self):
        pass
    def ComputeRandomVector(self):
        pass

    #Rotates smoothly the wind
    def RotateWind(self, strength):
        newWind = self.Wind
        rotation = rnd.uniform(-strength, strength)
        newWind.Rotate(rotation, rg.Vector3d(0,0,1))
        self.Wind = newWind
    
    #Changes smootly the wind strength
    def ComputeWindSpeed(self, strength):
        change = rnd.uniform(-strength, strength)
        self.WindSpeed += change

    #Updates the environment
    def Update(self):
        #self.RandomWindVector()
        #self.RandomVectorLength
        self.RotateWind(0.05)
        self.ComputeWindSpeed(0.1)
        self.SpawnFood(2, 200, 20)
        self.ComputeFoodLive(10)

##########################################################################################################

class BirdSystem:
    #Creating different types of agents
    #Realtime flocking optimization using R-Tree system.
    def __init__(self,agentCount):
        self.Birds = []
        self.Nests = []
   
        self.MinCount = agentCount / 2

         #Create New Nests
        NestCount = len(iAgentSpawnLocations)
        for i in range (0, NestCount):
            self.Nests.append(Nest(i, iAgentSpawnLocations[i]))


        #Check for inconsistencies
        if(NestCount < iNestCount):
            print ("Error iAgentSpawnLocations VS iNestCount -> See Guillaume")

    
        self._nestsAvailable = copy.copy(self.Nests)
        for i in range(0, iNestCount):
            nest = rnd.choice(self._nestsAvailable)
            self._nestsAvailable.remove(nest)
            for j in range(0, agentCount):
                self.Birds.append(Bird(nest))
   


    def Update(self):
        for bird in self.Birds:
            if bird.Alive == False: continue 
            bird.DieWithoutFood(600)
            bird.FindClosestFood()
            bird.Eat(50, 10)
            bird.ComputeRandomMovement(0.05)
            bird.ComputeAvoidGroundFloor(10, 30)
            bird.ComputeToFoodVector(7)
            bird.ComputeFlockingVector(iCohesionStrength, iAlignStrength)
            bird.ComputeWindVector(iWindSpeed)
            bird.ComputeUpliftRevolveVector(0.5, 3.0, 4.0)
            bird.ComputeAvoidPredatorVector(10)
            bird.ComputeBackToNest(10)
            bird.ComputeAvoidBrepsVector()
            bird.ComputeAvoidBoundingBoxVector()
        for bird in self.Birds:
            if bird.Alive == False: continue 
            bird.Update()
        bird.SpawnNewBird(iAgentSpawnLocations, 5)
         
##########################################################################################################

class PredatorSystem:
    def __init__(self, predatorCount):
        self.Predators = []
        self.Nests = []
        self.Count = len(self.Predators)
        self.MinCount = predatorCount / 2

        #Create New Nests
        NestCount = len(iPredatorSpawnLocations)
        for i in range (0, NestCount):
            self.Nests.append(Nest(i, iPredatorSpawnLocations[i]))
            print (i)
 
        for i in range(0, NestCount):
            nest = self.Nests[i]
            for j in range(0, predatorCount):
                self.Predators.append(Predator(nest))


    def Update(self):
        for predator in self.Predators:
            if predator.Alive == False: continue
            predator.Hungry(3, 200)
            predator.EatBird(5)
            predator.ComputeRandomMovement(0.2)
            predator.ComputeAvoidGroundFloor(1.0, 20)
            predator.SearchClosestBird(100)
            predator.WaitingForFood(2.0, 0.1, 0.2, 0.01)
            predator.ComputeUpliftRevolveVector(0.05, 0.4, 0.5)
            predator.ComputeWindVector(0.01)
            predator.ComputeAvoidBoundingBoxVector()
            predator.ComputeAvoidBrepsVector()
        for predator in self.Predators:
            if predator.Alive == False: continue
            predator.Update()
        predator.SpawnNewPredator(iPredatorSpawnLocations)

##########################################################################################################

##########################################################################################################

class Agent:
    def __init__(self, nest):
        self.Radius = 10
        self.Position = rg.Point3d(rnd.uniform(nest.Position.X - self.Radius,nest.Position.X + self.Radius),rnd.uniform(nest.Position.Y - self.Radius,nest.Position.Y + self.Radius),rnd.uniform(nest.Position.Z,nest.Position.Z + self.Radius*2))
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)
        self.Nest = nest
        self.History = [self.Position]

    #Small random vector
    def ComputeRandomMovement(self, value):
        randomVec = rg.Vector3d(rnd.uniform(-value, value),rnd.uniform(-value, value),rnd.uniform(-value, value))
        self.DesiredVelocity = randomVec

    #Agents avoid groundfloor
    def ComputeAvoidGroundFloor(self, strength, detectionLength):
        if self.Position.Z < 0.5:
            self.Alive = False
            self.Display = "Dead"
        elif self.Position.Z < detectionLength:
            #uplift = (detectionLength**2- - self.Position.Z**2)*strength
            uplift = (detectionLength / self.Position.Z -0.99) * strength
            self.DesiredVelocity += rg.Vector3d(0, 0, uplift)

    #Agents avoid obstacles
    def ComputeAvoidBrepsVector(self, strength):
        for obstacle in iObstacleBreps:
            projectedPt = obstacle.ClosestPoint(self.Position)
            # Mirror Method
            objectCollide = projectedPt - self.Position
            objectDistance = objectCollide.Length
            if objectDistance < 1:
                self.Alive = False
                self.Display = "Dead"
                continue 
            if objectDistance < iDetectonDistance:
                objectCollide.Unitize()
                objectCollide *= -(1- (objectDistance / iDetectonDistance))**2 #better formula?
                #objectCollide *= (iDetectonDistance / objectDistance - 0.99)**2
                objectCollide.Unitize()
                self.DesiredVelocity += (objectCollide*strength)

    def ComputeAvoidBoundingBoxVector(self):
        projectedPt = iBoundingBox.ClosestPoint(self.Position)
        objectCollide = projectedPt - self.Position
        objectDistance = objectCollide.Length
        if 0 < objectDistance <= 2:
            obstacleFaces = iBoundingBox.Faces 
            for face in obstacleFaces: #Determine which face is closest
                closestPoint = face.ClosestPoint(self.Position)
                surfacePoint = rg.Surface.PointAt(face, closestPoint[1], closestPoint[2])
                distance = surfacePoint.DistanceTo(self.Position)
                if 0 < distance <= 1:  #checking if agent is on face
                    plane = face.FrameAt(0.5, 0.5)
                    zAxis = plane[1].ZAxis
                    zAxis.Unitize()
                    self.Velocity.Unitize()
                    self.DesiredVelocity += (-zAxis*(2*objectDistance**2)*1000 + self.Velocity*(2*objectDistance**2))  *100

    def ComputeAvoidBrepsVector(self):
        for obstacle in iObstacleBreps:
            projectedPt = obstacle.ClosestPoint(self.Position)
            objectCollide = projectedPt - self.Position
            objectDistance = objectCollide.Length
            if 0 < objectDistance <= 20:
                obstacleFaces = obstacle.Faces 
                for face in obstacleFaces: #Determine which face is closest
                    closestPoint = face.ClosestPoint(self.Position)
                    surfacePoint = rg.Surface.PointAt(face, closestPoint[1], closestPoint[2])
                    distance = surfacePoint.DistanceTo(self.Position)
                    if 0 < distance/objectDistance <= 10:  #checking if agent is on face
                        plane = face.FrameAt(0.5, 0.5)
                        zAxis = plane[1].ZAxis
                        zAxis.Unitize()
                        self.Velocity.Unitize()
                        self.DesiredVelocity += (zAxis*(2*objectDistance**2) + self.Velocity*(2*objectDistance**2))  *20

    def ComputeWindVector(self, strength):
        self.DesiredVelocity += (environmentSystem.Wind * strength)

    #Computes wind on agents
    def ComputeWindVector(self, strength):
        self.DesiredVelocity += (environmentSystem.Wind * strength)
        
    #Not used!!!!
    def ComputeUpliftVector(self, strength): 
        for i in range(len(iUpliftArea)):
            testInside = iUpliftArea[i].IsPointInside(self.Position, 0.1, True)
            if testInside == True:
                upliftVector = rg.Vector3d(0.0, 0.0, iUpliftStrength[i]) 
                upliftVecotr.Unitize()
                self.DesiredVelocity += (upliftVector*strength)

    #Computes uplift on agents
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
                #vector.Unitize()
                self.DesiredVelocity += (vector*strength)

#########################################################################################################          

#########################################################################################################          



class Bird(Agent):
    def __init__(self, nest):
        Agent.__init__(self, nest)
        self.Alive = True
        self.WithoutFood = "Not initialized"
        self.Full = False
        self.Hungry = True
        self.FullValue = 1000
        self.HungryValue = 500

     
        if rnd.choice([0, 1]) > 0.5:
            self.Rotation = True
        else: 
            self.Rotation = False
        self.Display = self.WithoutFood

    #SpawnsNewBirds if to much died
    def SpawnNewBird(self, locations, amount):
        aliveCount = 0
        for bird in birdSystem.Birds:
            if bird.Alive == True:
                aliveCount += 1
        if aliveCount < birdSystem.MinCount: 
            location = rnd.choice(birdSystem.Nests)
            for i in range(amount):
                birdSystem.Birds.append(Bird(location))


    def ComputeFoodToNestVector(self):
        pass

    #Searches for closest food
    def FindClosestFood(self):
        closestFood = 0
        foodDistance = 10000
        lenFood = len(environmentSystem.Food)
        for i in range(lenFood):
            if self.Position.DistanceTo(environmentSystem.Food[i]) < foodDistance:
                closestFood = environmentSystem.Food[i]
                foodDistance = self.Position.DistanceTo(environmentSystem.Food[i])
        self.ClosestFood = closestFood

    #Computes vector to closest food
    def ComputeToFoodVector(self, strength):
        #toFood = iFoodSource - self.Position
        if (self.ClosestFood != 0 and self.Hungry == True):
            toFood = self.ClosestFood - self.Position
            toFood.Unitize()
            toFood *= strength
            self.DesiredVelocity += toFood

    #Check if bird is full (not hungry anymore) and compute vector back to original Nest
    def ComputeBackToNest(self, Strength):
        if (self.Full == True):
            toNest = self.Nest.Position - self.Position
            toNest.Unitize()
            toNest *= Strength
            self.DesiredVelocity += toNest
        
    def ComputeFlockingVector(self, CohesionStrength, AlignStrength):

        
        _birdPositions = []
        _birdVelocities = []
        for bird in birdSystem.Birds:
            #Get Birds from same nest
            if bird.Nest == self.Nest:
                #Get current bird positions
                _birdPositions.append(bird.Position)
                #Get current velocities
                _birdVelocities.append(bird.Velocity)

        #Rtree -> get closest birds to current bird
        ClosestBirds = list(rg.RTree.Point3dClosestPoints(_birdPositions,[self.Position],iDetectonDistance))

        CohesionVec = rg.Vector3d(0.0,0.0,0.0)
        VelVec = rg.Vector3d(0.0,0.0,0.0)

        for closebird in ClosestBirds[0]:
            if _birdPositions[closebird] == self.Position: continue #Skip self
            goToBird = _birdPositions[closebird] - self.Position #Get vector between current bird and closestbirds
            distance = goToBird.Length #Calculate distance between closestbirds
            
            #Align
            BirdVel = _birdVelocities[closebird]
            BirdVel.Unitize()
            VelVec += BirdVel

            #Separate
            if (distance < iSeparationDistance):
                getAway = self.Position - _birdPositions[closebird]
                getAwayDistance = getAway.Length
                getAway.Unitize()
                getAway *= (1 - (getAwayDistance/iSeparationDistance))**2
                CohesionVec += getAway
            
            #Cohesion
            else:
                goToBird.Unitize()
                goToBird *= (CohesionStrength / (distance + 1))**2
                CohesionVec += goToBird
            
            CohesionVelocity = CohesionVec/ len(ClosestBirds[0])*CohesionStrength   #AverageCohesion vector
            AlignVelocity = VelVec / len(ClosestBirds[0])*AlignStrength             #Average Velocities
            self.DesiredVelocity += (CohesionVelocity + AlignVelocity)/2.0          

    def ComputeAvoidPredatorVector(self, strength):
        
        #Get current positions predator
        _PredatorPos = []
        for predator in predatorSystem.Predators:
            _PredatorPos.append(predator.Position)
        
        #FindClosest predators using RTree's
        ClosestPredators = list(rg.RTree.Point3dClosestPoints(_PredatorPos,[self.Position],iDetectonDistance*1.5))
       
        #Getaway from predator vector
        for closepredator in ClosestPredators[0]:
            getAway = self.Position - _PredatorPos[closepredator]
            distance = getAway.Length
            if distance < iDetectonDistance*2:
                getAway.Unitize()
                self.DesiredVelocity += (getAway*strength)

    #def ComputeGroupAvoidPredatorVector(self, strength):
    #    for predator in predatorSystem.Predators:
    #        if predator.ClosestBird == "No one": continue
    #        birdIndex = predator.ClosestBird
    #        huntedBird = birdSystem.Birds[birdIndex].Position
    #        getAway = huntedBird - predator.Position
    #        if getAway.Length < iDetectonDistance:
    #            groupIndices = list(rg.RTree.Point3dClosestPoints(birdPositions,[huntedBird],iDetectonDistance))
    #            allIndices = []
    #            for index in groupIndices[0]:
    #                allIndices.append(index)
    #            allIndices.append(index)
    #            getAway.Unitize()
    #            for index in allIndices:
    #                birdSystem.Birds[index].DesiredVelocity += (getAway*strength)


### Food

    #Bird dies without food
    def DieWithoutFood(self, timeToDeath):
        if self.WithoutFood == "Not initialized":
            self.WithoutFood = timeToDeath
        if self.WithoutFood < 1:
            self.Alive = False
            self.Display = "Dead"
        else:
            self.WithoutFood -= 1

    #Bird eats food
    def Eat(self, eatDistance, addTimeToDeath):
        if self.ClosestFood != 0:
            if self.Position.DistanceTo(self.ClosestFood) < eatDistance:
                self.WithoutFood += addTimeToDeath

            #Define properties for back to Nest
            if (self.WithoutFood > self.FullValue and self.Full == False):
                self.Full = True
                self.Hungry = False

            #Define properties for food hunting
            if (self.WithoutFood < self.HungryValue and self.Hungry == False) :
                self.Full = False
                self.Hungry = True


############################################# Update

    #Updates bird
    def Update(self):
        self.Velocity = (1-iAgentVelocity) * self.Velocity + iAgentVelocity * self.DesiredVelocity
        if (self.Velocity.Length > 0.1):
            self.Velocity.Unitize()
            self.Velocity *= iSpeed
            self.Position += self.Velocity
            self.History.append(self.Position)
            if self.Display != "Dead":
                self.Display = self.WithoutFood
    pass

##########################################################################################################

class Predator(Agent):
    def __init__(self,nest):
        Agent.__init__(self, nest)
        self.Alive = True
        self.Hunting = True
        self.IsHungry = "Not initialized"
        self.ClosestBird = "No one"
        #True is positive, false is negative rotation
        if rnd.choice([0, 1]) > 0.5:
            self.Rotation = True
        else: 
            self.Rotation = False
        self.Display = self.IsHungry

###Defining Vectors

    #Spawns new predator if too much died
    def SpawnNewPredator(self, locations):
        aliveCount = 0
        for predator in predatorSystem.Predators:
            if predator.Alive == True:
                aliveCount += 1
        if aliveCount < predatorSystem.MinCount: 
            nest = rnd.choice(predatorSystem.Nests)
            predatorSystem.Predators.append(Predator(nest))

### Hunt

    def SearchClosestBird(self, searchStrength):
        if self.Hunting == True:
            #Current bird positions/velocities
            _birdPositions = []
            _birdVelocities = []
            _birdAlive = []
            for bird in birdSystem.Birds:
                _birdPositions.append(bird.Position)
                _birdVelocities.append(bird.Velocity)
                _birdAlive.append(bird.Alive)
            
            #init values
            myPosition = self.Position
            closestBird = rg.Point3d(0.0,0.0,0.0)
            closestBirdVel = rg.Vector3d(0.0,0.0,0.0)
            smallestDistance = iPredatorRadius

            #Find position and velocity of closest bird
            for i in range(len(_birdPositions)):
                if _birdAlive[i] == False: continue
                disToBird = myPosition.DistanceTo(_birdPositions[i])
                if disToBird < smallestDistance:
                    closestBird = _birdPositions[i]
                    closestBirdVel = _birdVelocities[i]
                    smallestDistance = disToBird
                    self.ClosestBird = i

            #If closest bird is within reach, hunt it and increase velocity
            if smallestDistance < iPredatorRadius:
                targetVec = closestBird - self.Position
                dis = targetVec.Length
                targetVec.Unitize()
                targetVec *= (searchStrength / (dis + 1))**2
                self.DesiredVelocity += targetVec
                self.Hunting = True
            else:
                self.Hunting = False
                self.ClosestBird = "No one"

    #If predator is not hungry, he waits till he's hungry again
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

### Eat

    #Computes if prpedator is hungry
    def Hungry(self, amountOfFood, restTime):
        if self.IsHungry == "Not initialized":
            self.IsHungry = amountOfFood
            self.Hunting = True
        if self.IsHungry <= 0:
            self.IsHungry -= 1
            self.Hunting = False
        if self.IsHungry == -(restTime):
            self.IsHungry = amountOfFood
            self.Hunting = True

    #Predator eats a bird if he's in given distance
    def EatBird(self, eatDistance):
        if self.IsHungry > 0:
            if self.ClosestBird != "No one":
                if self.Position.DistanceTo(birdSystem.Birds[self.ClosestBird].Position) < eatDistance:
                    birdSystem.Birds[self.ClosestBird].Alive = False
                    birdSystem.Birds[self.ClosestBird].Display = "Dead"
                    self.IsHungry -= 1

############################################# Update

    #Updates predator
    def Update(self):
        self.Velocity = (1-iPredatorVelocity) * self.Velocity + iPredatorVelocity * self.DesiredVelocity
        if (self.Velocity.Length > 0.1):
            self.Velocity.Unitize()
            self.Velocity *= iPredatorSpeed
            self.Position += self.Velocity
            self.History.append(self.Position)
            if self.Display != "Dead":
                self.Display = self.IsHungry
    pass

### Main script ###

#Define Nest Class available for Birds and Predators
class Nest:
    def __init__(self, id, position):
        self.ID = id
        self.Position = position

        
##########################################################################################################


#Initialize BirdSystem & PredatorSystem
if iEnabled == True or iReset:
    if iReset or "birdSystem" not in globals():
        environmentSystem = EnvironmentSystem()
        birdSystem = BirdSystem(iAgentCount)
        predatorSystem = PredatorSystem(iPredatorCount)
    else: 
#Updates the systems
        environmentSystem.Update()
        birdSystem.Update()
        predatorSystem.Update()

birdHistory = []
birdPositions = []
birdTag = []

predatorHistory = []
predatorPositions = []
predatorTarget = []
predatorTag = []
birdVelocities = []

#Creating predator output
for predator in predatorSystem.Predators:
    predatorPositions.append(predator.Position)
    #predatorHistory.append(rg.PolylineCurve(predator.History))
    predatorHistory.append(rg.NurbsCurve.Create(False, 3, predator.History)) 
    predatorTag.append(predator.Display)
    if predator.ClosestBird != "No one":
        predatorTarget.append(birdSystem.Birds[predator.ClosestBird].Position)
#Creating bird output
for bird in birdSystem.Birds:
    birdPositions.append(bird.Position)
    #birdHistory.append(rg.PolylineCurve(bird.History))
    birdVelocities.append(bird.Velocity)
    birdHistory.append(rg.NurbsCurve.Create(False, 3, bird.History))
    birdTag.append(bird.Display)

#Initialize ouput
oWind = environmentSystem.Wind
oFood = environmentSystem.Food
oFoodTag = environmentSystem.AmountFood

oWind = environmentSystem.Wind

oBirdHistory = birdHistory
oBirdPositions = birdPositions
oBirdTag = birdTag

oPredatorHistory = predatorHistory
oPredatorPositions = predatorPositions
oPredatorTag = predatorTag
oPredatorTarget = predatorTarget
obirdVelocities = birdVelocities

