###############################################
# ITECH 2020 Computational Design Thinking Final Assignment
# By Guillaume, Simon, Tim, Gabriel
###############################################
# 3D Printing Agent-Based BirdFlocking
###############################################

import Rhino.Geometry as rg

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


class Agent:
    def ComputeAvoidObstacleVector(self):
        pass

    def ComputeWindVector(self):
        pass

    def ComputeUpliftVector(self):
        pass


class BirdSystem:
    #Creating different types of agents
    #Realtime flocking optimization using R-Tree system.
    def __init__():
        pass

    def RecordHistory(self):
        pass

class PredatorSystem:
    def __init__():
        pass

class Bird(Agent):
    def __init__():
        pass

    def ComputeFoodToNestVector(self):
        pass

    def ComputeNestToFoodVector(self):
        pass

    def ComputeFlockingVector(self):
        pass

    def ComputeAvoidPredatorVector(self):
        pass

    def Die(self):
        pass

class Predator(Agent):
    def __init__():
        pass

    def ComputeBirdDensityVector(self):
        pass

    def ComputeRandomMovement(self):
        pass

    def EatBird(self):
        pass

