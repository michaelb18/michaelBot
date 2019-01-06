from math import sin, cos
import random

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator

from RLUtilities.GameInfo import GameInfo
from RLUtilities.Simulation import Car, Ball, Input
from RLUtilities.LinearAlgebra import vec3, dot
from RLUtilities.Maneuvers import Drive
from RLUtilities.Maneuvers import HalfFlip
import math
class State:
    CHASING=0
    SHADOWING=1
class Agent(BaseAgent):

    def __init__(self, name, team, index):
        self.index = index
        self.info = GameInfo(index, team)
        self.controls = SimpleControllerState()

        self.timer = 0.0
        self.timeout = 3.0

        self.action = None
        self.state = State.CHASING

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.info.read_packet(packet)

        if(math.sqrt(math.pow(self.info.opponents[0].pos[0]-self.info.my_goal.pos[0],2)+math.pow(self.info.opponents[0].pos[1]-self.info.my_goal.pos[1],2))<math.sqrt(math.pow(self.info.my_car.pos[0]-self.info.my_goal.pos[0],2)+math.pow(self.info.my_car.pos[1]-self.info.my_goal.pos[1],2))):
            self.state=State.SHADOWING
        if(self.state==State.CHASING):
            self.action=Drive(self.info.my_car,self.info.ball.pos,3000)
        elif(self.state==State.SHADOWING):
            x = -893
            if (self.info.opponents[0].pos[0] > 0):
                x = 893
            self.action=Drive(self.info.my_car.pos,vec3(self.info.my_goal.pos[0]+x,self.info.my_goal[1],self.info.my_goal[2]),math.sqrt(math.pow(self.info.ball.vel[0],2)+math.pow(self.info.ball.vel[0],2)))

        return self.controls