import math
import random

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator

from RLUtilities.GameInfo import GameInfo
from RLUtilities.Simulation import Car, Ball
from RLUtilities.LinearAlgebra import vec3, dot

from RLUtilities.Maneuvers import AerialTurn

class Agent(BaseAgent):

    def __init__(self, name, team, index):
        self.index = index
        self.info = GameInfo(index, team)
        self.controls = SimpleControllerState()

        self.timer = 0.0
        self.action = None

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.info.read_packet(packet)
        self.controls = SimpleControllerState()




        if self.action == None:
            self.action = AerialTurn(self.info.my_car)


        self.action.step(1.0 / 60.0)
        self.controls = self.action.controls

        self.timer += 1.0 / 60.0
        if self.action.finished or self.timer > 5.0:
            print("target:\n", self.action.target)
            print("theta:\n", self.action.car.theta)
            print()
            self.timer = 0.0
            self.action = None

        self.timer += 1.0 / 60.0
        return self.controls
