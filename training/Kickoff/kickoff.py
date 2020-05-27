
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.agents.base_agent import SimpleControllerState
from RLUtilities.GameInfo import GameInfo
from RLUtilities.LinearAlgebra import vec3
#import sys
#from os.path import dirname
#sys.path.append(dirname(dirname(dirname(__file__))))
from behaviors.Kickoff import Kickoff
from util import *
class KickoffAgent():

    def __init__(self, name, team, index):
        #This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.info = GameInfo(index, team)
        self.state = 'init'
        self.team = team
    def get_output(self, packet: GameTickPacket, fps) -> SimpleControllerState:

        self.info.read_packet(packet)
        if(self.state == 'init'):
            self.hit = Kickoff(info=self.info , fps = fps)
            self.state = 'run'
        #creating a basis for the field
        return self.hit.execute(self.info, fps).to_simple_controller_state()