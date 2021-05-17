# bustersAgents.py
# ----------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


import util
from game import Agent
from game import Directions
from wekaI import Weka
from keyboardAgents import KeyboardAgent
from util import manhattanDistance
import distanceCalculator
import inference
import busters

class NullGraphics:
    "Placeholder for graphics"
    def initialize(self, state, isBlue = False):
        pass
    def update(self, state):
        pass
    def pause(self):
        pass
    def draw(self, state):
        pass
    def updateDistributions(self, dist):
        pass
    def finish(self):
        pass

class KeyboardInference(inference.InferenceModule):
    """
    Basic inference module for use with the keyboard.
    """
    def initializeUniformly(self, gameState):
        "Begin with a uniform distribution over ghost positions."
        self.beliefs = util.Counter()
        for p in self.legalPositions: self.beliefs[p] = 1.0
        self.beliefs.normalize()

    def observe(self, observation, gameState):
        noisyDistance = observation
        emissionModel = busters.getObservationDistribution(noisyDistance)
        pacmanPosition = gameState.getPacmanPosition()
        allPossible = util.Counter()
        for p in self.legalPositions:
            trueDistance = util.manhattanDistance(p, pacmanPosition)
            if emissionModel[trueDistance] > 0:
                allPossible[p] = 1.0
        allPossible.normalize()
        self.beliefs = allPossible

    def elapseTime(self, gameState):
        pass

    def getBeliefDistribution(self):
        return self.beliefs


class BustersAgent:
    "An agent that tracks and displays its beliefs about ghost positions."

    def __init__( self, index = 0, inference = "ExactInference", ghostAgents = None, observeEnable = True, elapseTimeEnable = True):
        inferenceType = util.lookup(inference, globals())
        self.inferenceModules = [inferenceType(a) for a in ghostAgents]
        self.observeEnable = observeEnable
        self.elapseTimeEnable = elapseTimeEnable
        self.weka = Weka()
        self.weka.start_jvm()


    def registerInitialState(self, gameState):
        "Initializes beliefs and inference modules"
        import __main__
        self.display = __main__._display
        for inference in self.inferenceModules:
            inference.initialize(gameState)
        self.ghostBeliefs = [inf.getBeliefDistribution() for inf in self.inferenceModules]
        self.firstMove = True

    def observationFunction(self, gameState):
        "Removes the ghost states from the gameState"
        agents = gameState.data.agentStates
        gameState.data.agentStates = [agents[0]] + [None for i in range(1, len(agents))]
        return gameState

    def getAction(self, gameState):
        "Updates beliefs, then chooses an action based on updated beliefs."
        #for index, inf in enumerate(self.inferenceModules):
        #    if not self.firstMove and self.elapseTimeEnable:
        #        inf.elapseTime(gameState)
        #    self.firstMove = False
        #    if self.observeEnable:
        #        inf.observeState(gameState)
        #    self.ghostBeliefs[index] = inf.getBeliefDistribution()
        #self.display.updateDistributions(self.ghostBeliefs)
        return self.chooseAction(gameState)

    def chooseAction(self, gameState):
        "By default, a BustersAgent just stops.  This should be overridden."
        return Directions.STOP
    def calculateFoodPositions(self, state):
        """
        Returns the food positions
        """
        foodPos = [9999999] * state.getNumFood()
        if(state.getNumFood() > 0):
            # minDistance = 900000
            # pacmanPosition = state.getPacmanPosition()
            counter = 0
            for i in range(state.data.layout.width):
                for j in range(state.data.layout.height):
                    if state.hasFood(i, j):
                        foodPos[counter] = (i,j)
                        counter += 1
                        # distance = util.manhattanDistance(pacmanPosition, foodPosition)
                        # if distance < minDistance:
                            # minDistance = distance
            return foodPos

        else:
            return None

    def bestActions( self, state ):
        # Read variables from state
        legalActions = state.getLegalActions(0)
        # No tiene sentido que incluyamos Stop, buscamos que termine lo antes posible
        if 'Stop' in legalActions:
            legalActions.remove("Stop")
        pos = state.getPacmanPosition()                 #posicion pacman
        ghosts_pos = state.getGhostPositions()          #posicion fantasmas

        bestGhostAction = ['None'] * 4            #mejor accion para cada uno de los ghosts 
        distancesToGhosts = [10000] * 4     #distancia del pacman a cada uno de los ghosts 


        #Encuentra la mejor accion del Pacman para cada fantasma
        for i in range(0,4):
            mini = 9999999999
            if (state.getLivingGhosts()[i+1]):
                aux = 0
                if 'North' in legalActions:
                    aux = self.distancer.getDistance((pos[0],pos[1]+1), ghosts_pos[i])
                    if mini > aux: 
                        bestGhostAction[i] = 'North'
                        distancesToGhosts[i] = aux
                        mini = aux
                if 'East' in legalActions:
                    aux = self.distancer.getDistance((pos[0]+1,pos[1]), ghosts_pos[i])
                    if mini > aux:
                        bestGhostAction[i] = 'East'
                        distancesToGhosts[i] = aux
                        mini = aux
                if 'West' in legalActions:
                    aux = self.distancer.getDistance((pos[0]-1,pos[1]), ghosts_pos[i])
                    if mini > aux: 
                        bestGhostAction[i] = 'West'
                        distancesToGhosts[i] = aux
                        mini = aux
                if 'South' in legalActions:
                    aux = self.distancer.getDistance((pos[0],pos[1]-1), ghosts_pos[i])
                    if mini > aux: 
                        bestGhostAction[i] = 'South'
                        distancesToGhosts[i] = aux
                        mini = aux
        
        bestScore = min(distancesToGhosts)          #distancia al fantasma
        
                   

        bestAction=[]                                       #mejor accion
        #Elige la mejor entre la mejor accion para cada fantasma/comida
        for action, distance in zip(bestGhostAction, distancesToGhosts):
            if distance == bestScore:
                if action != 'None':
                    bestAction = [action]
                else:
                    bestAction = 'None'

        
        aux = distancesToGhosts[:]
        aux.sort()
        for i in range(0,4):
            index = distancesToGhosts.index(aux[i])
            distancesToGhosts[index] = i

        return bestAction[0], bestGhostAction, distancesToGhosts

        
    def printLineData(self, gameState):
        s = []

        s.append(str(gameState.getPacmanPosition()[0])) #p_x----
        s.append(str(gameState.getPacmanPosition()[1])) #p_y----
        if "West" in gameState.getLegalPacmanActions():     #go_west----
            s.append("T")
        else:
            s.append("F")
        if "East" in gameState.getLegalPacmanActions():     #go_east----
            s.append("T")
        else:
            s.append("F")
        if "North" in gameState.getLegalPacmanActions():    #go_north----
            s.append("T")
        else:
            s.append("F")
        if "South" in gameState.getLegalPacmanActions():    #go_south----
            s.append("T")
        else:
            s.append("F")
        
        for i in range(1, gameState.getNumAgents()): #g1,g2,g3,g4----
            if gameState.getLivingGhosts()[i]:
                s.append("T")  # fantasma vivo
            else:
                s.append("F")  # fantasma comido
        missing_ghosts = abs(gameState.getNumAgents() - 1 - 4)

        if missing_ghosts > 0:
            for i in range(0, missing_ghosts):
                s.append("F")  # Si hay menos de 4 fantasmas, el resto se indica con N que no existe

        for i in range(1, 5):                        #g1x,g1y, g2x,g2y, g3x,g3y, g4x,g4y----
            if i <= gameState.getNumAgents() - 1:
                s.append(gameState.getGhostPositions()[i - 1][0])
                s.append(gameState.getGhostPositions()[i - 1][1])
            else:
                s.append(-1)
                s.append(-1)

        bestAction, bestActions, distancesToGhosts = self.bestActions(gameState)
        for i in range(0, 4):   #distance to ghosts----
            if ((i <= gameState.getNumAgents() - 2) and distancesToGhosts[i]<10000):
                s.append(distancesToGhosts[i])  #Si el fantasma ya fue comido, su distancia es 1000
            else:
                s.append(-1)                                # Si hay menos de 4 fantasmas, se indica una distancia falsa de -1

        for i in range(0, 4):
            if gameState.getGhostDirections().get(i) != None:
                s.append(gameState.getGhostDirections().get(i)) 
            else:
                s.append('Stop')
        
        for i in bestActions:
            s.append(str(i))
   
        s.append(str(gameState.getScore())) #score----

        # s.append(str(gameState.data.agentStates[0].getDirection())) #dir
        direc = gameState.data.agentStates[0].getDirection()
        score = gameState.getScore()

        output = s[0]
        for i in s[1::]:
            output = output + "," + str(i)

        return direc, score, output



class BustersKeyboardAgent(BustersAgent, KeyboardAgent):
    "An agent controlled by the keyboard that displays beliefs about ghost positions."

    def __init__(self, index = 0, inference = "KeyboardInference", ghostAgents = None):
        KeyboardAgent.__init__(self, index)
        BustersAgent.__init__(self, index, inference, ghostAgents)

    def getAction(self, gameState):
        return BustersAgent.getAction(self, gameState)

    def chooseAction(self, gameState):
        return KeyboardAgent.getAction(self, gameState)
        
    

from distanceCalculator import Distancer
from game import Actions
from game import Directions
import random, sys

'''Random PacMan Agent'''
class RandomPAgent(BustersAgent):

    def registerInitialState(self, gameState):
        BustersAgent.registerInitialState(self, gameState)
        self.distancer = Distancer(gameState.data.layout, False)
        
    ''' Example of counting something'''
    def countFood(self, gameState):
        food = 0
        for width in gameState.data.food:
            for height in width:
                if(height == True):
                    food = food + 1
        return food
    
    ''' Print the layout'''  
    def printGrid(self, gameState):
        table = ""
        ##print(gameState.data.layout) ## Print by terminal
        for x in range(gameState.data.layout.width):
            for y in range(gameState.data.layout.height):
                food, walls = gameState.data.food, gameState.data.layout.walls
                table = table + gameState.data._foodWallStr(food[x][y], walls[x][y]) + ","
        table = table[:-1]
        return table
        
    def chooseAction(self, gameState):
        move = Directions.STOP
        legal = gameState.getLegalActions(0) ##Legal position from the pacman
        move_random = random.randint(0, 3)
        if   ( move_random == 0 ) and Directions.WEST in legal:  move = Directions.WEST
        if   ( move_random == 1 ) and Directions.EAST in legal: move = Directions.EAST
        if   ( move_random == 2 ) and Directions.NORTH in legal:   move = Directions.NORTH
        if   ( move_random == 3 ) and Directions.SOUTH in legal: move = Directions.SOUTH
        return move
        
class GreedyBustersAgent(BustersAgent):
    "An agent that charges the closest ghost."

    def registerInitialState(self, gameState):
        "Pre-computes the distance between every two points."
        BustersAgent.registerInitialState(self, gameState)
        self.distancer = Distancer(gameState.data.layout, False)

    def chooseAction(self, gameState):
        """
        First computes the most likely position of each ghost that has
        not yet been captured, then chooses an action that brings
        Pacman closer to the closest ghost (according to mazeDistance!).

        To find the mazeDistance between any two positions, use:
          self.distancer.getDistance(pos1, pos2)

        To find the successor position of a position after an action:
          successorPosition = Actions.getSuccessor(position, action)

        livingGhostPositionDistributions, defined below, is a list of
        util.Counter objects equal to the position belief
        distributions for each of the ghosts that are still alive.  It
        is defined based on (these are implementation details about
        which you need not be concerned):

          1) gameState.getLivingGhosts(), a list of booleans, one for each
             agent, indicating whether or not the agent is alive.  Note
             that pacman is always agent 0, so the ghosts are agents 1,
             onwards (just as before).

          2) self.ghostBeliefs, the list of belief distributions for each
             of the ghosts (including ghosts that are not alive).  The
             indices into this list should be 1 less than indices into the
             gameState.getLivingGhosts() list.
        """
        pacmanPosition = gameState.getPacmanPosition()
        legal = [a for a in gameState.getLegalPacmanActions()]
        livingGhosts = gameState.getLivingGhosts()
        livingGhostPositionDistributions = \
            [beliefs for i, beliefs in enumerate(self.ghostBeliefs)
             if livingGhosts[i+1]]
        return Directions.EAST

class BasicAgentAA(BustersAgent):

    def registerInitialState(self, gameState):
        BustersAgent.registerInitialState(self, gameState)
        self.distancer = Distancer(gameState.data.layout, False)
        self.countActions = 0
        
    ''' Example of counting something'''
    def countFood(self, gameState):
        food = 0
        for width in gameState.data.food:
            for height in width:
                if(height == True):
                    food = food + 1
        return food
    
    ''' Print the layout'''  
    def printGrid(self, gameState):
        table = ""
        #print(gameState.data.layout) ## Print by terminal
        for x in range(gameState.data.layout.width):
            for y in range(gameState.data.layout.height):
                food, walls = gameState.data.food, gameState.data.layout.walls
                table = table + gameState.data._foodWallStr(food[x][y], walls[x][y]) + ","
        table = table[:-1]
        return table

    def printInfo(self, gameState):
        print "---------------- TICK ", self.countActions, " --------------------------"
        # Dimensiones del mapa
        width, height = gameState.data.layout.width, gameState.data.layout.height
        print "Width: ", width, " Height: ", height
        # Posicion del Pacman
        print "Pacman position: ", gameState.getPacmanPosition()
        # Acciones legales de pacman en la posicion actual
        print "Legal actions: ", gameState.getLegalPacmanActions()
        # Direccion de pacman
        print "Pacman direction: ", gameState.data.agentStates[0].getDirection()
        # Numero de fantasmas
        print "Number of ghosts: ", gameState.getNumAgents() - 1
        # Fantasmas que estan vivos (el indice 0 del array que se devuelve corresponde a pacman y siempre es false)
        print "Living ghosts: ", gameState.getLivingGhosts()
        # Posicion de los fantasmas
        print "Ghosts positions: ", gameState.getGhostPositions()
        # Direciones de los fantasmas
        print "Ghosts directions: ", [gameState.getGhostDirections().get(i) for i in range(0, gameState.getNumAgents() - 1)]
        # Distancia de manhattan a los fantasmas
        print "Ghosts distances: ", gameState.data.ghostDistances
        # Puntos de comida restantes
        print "Pac dots: ", gameState.getNumFood()
        # Distancia de manhattan a la comida mas cercada
        print "Distance nearest pac dots: ", gameState.getDistanceNearestFood()
        # Paredes del mapa
        print "Map:  \n", gameState.getWalls()
        # Puntuacion
        print "Score: ", gameState.getScore()

    

    def getDistance_with_wall(self, pos1, pos2):
        """
        The getDistance function is the only one you'll need after you create the object.
        """
        if self._distances == None:
            return manhattanDistance(pos1, pos2)
        if isInt(pos1) and isInt(pos2):
            return self.getDistanceOnGrid(pos1, pos2)
        pos1Grids = getGrids2D(pos1)
        pos2Grids = getGrids2D(pos2)
        bestDistance = self.default
        for pos1Snap, snap1Distance in pos1Grids:
            for pos2Snap, snap2Distance in pos2Grids:
                gridDistance = self.getDistanceOnGrid(pos1Snap, pos2Snap)
                distance = gridDistance + snap1Distance + snap2Distance
                if bestDistance > distance:
                    bestDistance = distance
        return bestDistance

    def calculateDistribution( self, state ):
        # Read variables from state
        numghosts = state.getNumAgents()-1
        legalActions = state.getLegalActions(0)
        # No tiene sentido que incluyamos Stop, buscamos que termine lo antes posible
        legalActions.remove("Stop")

        pos = state.getPacmanPosition()                 #posicion pacman
        ghosts_pos = state.getGhostPositions()          #posicion fantasmas

        bestGhostAction = [None] * numghosts            #mejor accion para cada uno de los ghosts 
        distancesToGhosts = [999999999] * numghosts     #distancia del pacman a cada uno de los ghosts 


        #Encuentra la mejor accion del Pacman para cada fantasma
        for i in range(0,numghosts):
            mini = 9999999999
            if (state.getLivingGhosts()[i+1]):
                aux = 0
                if 'North' in legalActions:
                    aux = self.distancer.getDistance((pos[0],pos[1]+1), ghosts_pos[i])
                    if mini > aux: 
                        bestGhostAction[i] = 'North'
                        distancesToGhosts[i] = aux
                        mini = aux
                if 'East' in legalActions:
                    aux = self.distancer.getDistance((pos[0]+1,pos[1]), ghosts_pos[i])
                    if mini > aux:
                        bestGhostAction[i] = 'East'
                        distancesToGhosts[i] = aux
                        mini = aux
                if 'West' in legalActions:
                    aux = self.distancer.getDistance((pos[0]-1,pos[1]), ghosts_pos[i])
                    if mini > aux: 
                        bestGhostAction[i] = 'West'
                        distancesToGhosts[i] = aux
                        mini = aux
                if 'South' in legalActions:
                    aux = self.distancer.getDistance((pos[0],pos[1]-1), ghosts_pos[i])
                    if mini > aux: 
                        bestGhostAction[i] = 'South'
                        distancesToGhosts[i] = aux
                        mini = aux
        
        bestScore = min(distancesToGhosts)                               #distancia al fantasma o comida mas cercano
        
        bestAction=[]                                       #mejor accion

        #Elige la mejor entre la mejor accion para cada fantasma/comida
        for action, distance in zip(bestGhostAction, distancesToGhosts):
            if distance == bestScore:
                if action != None:
                    bestAction = [action]

        return bestAction[0]
        
    def chooseAction(self, gameState):

        action, bestGhostAction, distancesToGhosts = self.bestActions(gameState)
        
        #------------------------------FASE 4--------------------------------------------------#
        direc, score, output = self.printLineData(gameState)
        l = output.split(",")
        l.append(str(score))
        
        x=[]

        #Seleccionar los atributos que usamos para cada instancia (no incluir la clase)
        for i in range(0, len(l)):
            if ((i>=2 and i<=9) or (i>=18 and i<=29)):
                x.append((l[i]))
        
        a = self.weka.predict("j48_tutorial1.model", x, "fase4.arff")

        if (a not in gameState.getLegalActions()):
            a=random.choice(gameState.getLegalActions())
        #----------------------------------------------------------------------------------------#

        return a
