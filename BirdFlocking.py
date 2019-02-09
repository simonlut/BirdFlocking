###############################################
# ITECH 2020 Computational Design Thinking Final Assignment
# By Guillaume, Simon, Tim, Gabriel
###############################################
# 3D Printing Agent-Based BirdFlocking
###############################################





#Todo Guillaume :
        #-If waiting for food, searches closest uplift and goes there - Guillaume
        #(def WaitingForFood - goest first to uplift, than circles) -> Is it Usefull?
    #Presentation:
        #Diagram Inheritance/Classes and description of the definitions - Guillaume (sunday)





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

    def SpawnFood(self, amount, initFood):
        if self.Food == "Not initialized":
            self.Food = []
            self.AmountFood = []
            for i in range(amount):
                newLocation = rnd.choice(iFoodSpawnLocations)
                self.Food.append(newLocation)
                self.AmountFood.append(initFood)
        if len(self.Food) < amount/2:
                newLocation = rnd.choice(range(len(iFoodSpawnLocations)))
                self.Food.append(newLocation)
                self.AmountFood.append(initFood)

    def InitializeFood(self, amount): #Not used
        if self.AmountFood == "Not initialized":
            self.AmountFood = []
            for food in self.Food:
                self.AmountFood.append(amount)
        for i in range(len(self.AmountFood)):
            if self.AmountFood[i] <= 0:
                del self.AmountFood[i]
                del self.Food[i]

    def ComputeFoodLive(self, eatDistance):
        if "birdSystem" in globals():
            _birdPositions = []
            for bird in birdSystem.Birds:
                _birdPositions.append(bird.Position)
            for i in range(len(self.Food)):
                closestBirds = list(rg.RTree.Point3dClosestPoints(_birdPositions,[self.Food[i]], eatDistance))
                self.AmountFood[i] -= len(closestBirds[0])
                if self.AmountFood[i] <= 0:
                    del self.AmountFood[i]
                    del self.Food[i]      


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

    def RotateWind(self, strength):
        newWind = self.Wind
        rotation = rnd.uniform(-strength, strength)
        newWind.Rotate(rotation, rg.Vector3d(0,0,1))
        self.Wind = newWind

    def ComputeWindSpeed(self, strength):
        change = rnd.uniform(-strength, strength)
        self.WindSpeed += change

    def RandomWindVector(self): #Not used
        randomVecWind = rg.Vector3d(rnd.uniform(-1.0, 1.0), rnd.uniform(-1.0, 1.0), 0.0)
        #newVecWind = rg.Vector3d.Rotate(self.Wind, rnd.uniform(0.1, 1), rg.Vector3d(0,0,1))
        newVecWind = (8/10) * self.Wind + (2/10) * randomVecWind
        newVecWind.Unitize()
        self.Wind = newVecWind

    def RandomVectorLength(self):#Not used
        newVecWind = self.Wind * rnd.uniform(self.Wind.Length/2, self.Wind.Length*1.5)
        self.Wind = newVecWind
        print self.Wind.Length

    def Update(self):
        #self.RandomWindVector()
        #self.RandomVectorLength
        self.RotateWind(0.05)
        self.ComputeWindSpeed(0.1)
        self.SpawnFood(2, 20)
        self.ComputeFoodLive(5)

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
            bird.ComputeToFoodVector(10)
            bird.ComputeFlockingVector()
            bird.ComputeWindVector(0.2)
            #bird.ComputeAvoidObstacleVector() #Not used!!!
            bird.ComputeAvoidBrepsVector(100)
            #bird.ComputeUpliftVector()  #Not used!!!
            bird.ComputeUpliftRevolveVector(0.5, 3.0, 4.0)
            bird.ComputeGroupAvoidPredatorVector(10)
            bird.ComputeAvoidPredatorVector(10)
            bird.ComputeBackToNest(10)
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
            predator.SearchClosestBird()
            predator.WaitingForFood(2.0, 0.1, 0.2, 0.01)
            predator.ComputeAvoidBrepsVector(50)
            predator.ComputeUpliftRevolveVector(0.05, 0.4, 0.5)
            predator.ComputeWindVector(0.01)
        for predator in self.Predators:
            if predator.Alive == False: continue
            predator.Update()
        predator.SpawnNewPredator(iPredatorSpawnLocations)


##########################################################################################################

class Agent:
    def __init__(self, nest):
        self.Radius = 10
        self.Position = rg.Point3d(rnd.uniform(nest.Position.X - self.Radius,nest.Position.X + self.Radius),rnd.uniform(nest.Position.Y - self.Radius,nest.Position.Y + self.Radius),rnd.uniform(nest.Position.Z,nest.Position.Z + self.Radius*2))
        self.Velocity = rg.Vector3d(0.0, 0.0, 0.0)
        self.Nest = nest
        self.History = [self.Position]


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

    def ComputeAvoidGroundFloor(self, strength, detectionLength):
        if self.Position.Z < 0.5:
            self.Alive = False
            self.Display = "Dead"
        elif self.Position.Z < detectionLength:
            #uplift = (detectionLength**2- - self.Position.Z**2)*strength
            uplift = (detectionLength / self.Position.Z -0.99) * strength
            self.DesiredVelocity += rg.Vector3d(0, 0, uplift)


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

    def ComputeWindVector(self, strength):
        self.DesiredVelocity += (environmentSystem.Wind * strength)

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
                #vector.Unitize()
                self.DesiredVelocity += (vector*strength)
    pass


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

    def FindClosestFood(self):
        closestFood = 0
        foodDistance = 10000
        lenFood = len(environmentSystem.Food)
        for i in range(lenFood):
            if self.Position.DistanceTo(environmentSystem.Food[i]) < foodDistance:
                closestFood = environmentSystem.Food[i]
                foodDistance = self.Position.DistanceTo(environmentSystem.Food[i])
        self.ClosestFood = closestFood

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
        pass



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

    def ComputeGroupAvoidPredatorVector(self, strength):
        for predator in predatorSystem.Predators:
            if predator.ClosestBird == "No one": continue
            birdIndex = predator.ClosestBird
            huntedBird = birdSystem.Birds[birdIndex].Position
            getAway = huntedBird - predator.Position
            if getAway.Length < iDetectonDistance:
                groupIndices = list(rg.RTree.Point3dClosestPoints(birdPositions,[huntedBird],iDetectonDistance))
                allIndices = []
                for index in groupIndices[0]:
                    allIndices.append(index)
                allIndices.append(index)
                getAway.Unitize()
                for index in allIndices:
                    birdSystem.Birds[index].DesiredVelocity += (getAway*strength)


### Food

    def DieWithoutFood(self, timeToDeath):
        if self.WithoutFood == "Not initialized":
            self.WithoutFood = timeToDeath
        if self.WithoutFood < 1:
            self.Alive = False
            self.Display = "Dead"
        else:
            self.WithoutFood -= 1

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
    #Defining Vectors

    def SpawnNewPredator(self, locations):
        aliveCount = 0
        for predator in predatorSystem.Predators:
            if predator.Alive == True:
                aliveCount += 1
        if aliveCount < predatorSystem.MinCount: 
            nest = rnd.choice(predatorSystem.Nests)
            predatorSystem.Predators.append(Predator(nest))

### Hunt

    def SearchClosestBird(self):
        if self.Hunting == True:
            position = self.Position
            closestBird = self.Position
            distance = iPredatorRadius
            for i in range(len(birdSystem.Birds)):
                if birdSystem.Birds[i].Alive == True:
                    dis = position.DistanceTo(birdSystem.Birds[i].Position)
                    if dis < distance:
                        closestBird = birdSystem.Birds[i].Position
                        distance = dis
                        self.ClosestBird = i
            if distance < iPredatorRadius:
                targetVec = closestBird - self.Position
                targetVec.Unitize()
                self.DesiredVelocity += targetVec
                self.Hunting = True
            else:
                self.Hunting = False
                self.ClosestBird = "No one"

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

    def EatBird(self, eatDistance):
        if self.IsHungry > 0:
            if self.ClosestBird != "No one":
                if self.Position.DistanceTo(birdSystem.Birds[self.ClosestBird].Position) < eatDistance:
                    birdSystem.Birds[self.ClosestBird].Alive = False
                    birdSystem.Birds[self.ClosestBird].Display = "Dead"
                    self.IsHungry -= 1

############################################# Update

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

##########################################################################################################

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
        environmentSystem.Update()
        birdSystem.Update()
        predatorSystem.Update()

birdHistory = []
birdPositions = []
birdTag = []

predatorHistory = []
predatorPositions = []
predatorTag = []

for predator in predatorSystem.Predators:
    predatorPositions.append(predator.Position)
    #predatorHistory.append(rg.PolylineCurve(predator.History))
    predatorHistory.append(rg.NurbsCurve.Create(False, 3, predator.History)) 
    predatorTag.append(predator.Display)


for bird in birdSystem.Birds:
    birdPositions.append(bird.Position)
    #birdHistory.append(rg.PolylineCurve(bird.History))
    birdHistory.append(rg.NurbsCurve.Create(False, 3, bird.History))
    birdTag.append(bird.Display)


oWind = environmentSystem.Wind

oBirdHistory = birdHistory
oBirdPositions = birdPositions
oBirdTag = birdTag

oPredatorHistory = predatorHistory
oPredatorPositions = predatorPositions
oPredatorTag = predatorTag

