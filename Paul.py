from rlbot.agents.base_agent import BaseAgent
import Bot
from training.Kickoff.kickoff import KickoffAgent
from RLUtilities.Maneuvers import Drive
from RLUtilities.GameInfo import GameInfo
from rlbot.agents.base_agent import GameTickPacket
import time
from util import *
'''
Paul, by Marvin GooseFairy and Brom

Mini-ReadMe:
Paul is a set of tools we made in an effort to approach strategy more efficently.
Paul works with paths, which are lines draw from point-to-point in order to accomplish some state
Events can also be added to paths, which control the path speed, jumps, etc. Events are NOT fully implemented

In the master branch most of these tools are undisturbed, in the shooter circles branch paths are
made of lines and circles and the bot is capeable of playing a game.
'''
import threading
from queue import Queue
# TODO: not important, but I should update this name if goose lets me
class Paul(BaseAgent):

    def initialize_agent(self):
        #self.gui = GUI() #Paul has a nice GUI that allows for drawing paths, creating and moving/editing events
        self.info = GameInfo( self.index, self.team )
        team = 0
        if self.team == 1:
            team = -1
        else:
            team = 1
        self.action_state = ''
        self.frames = 0
        self.yeet_start = time.time()
        self.yeet_kick_off = KickoffAgent( 'eat ass', self.info.team, self.info.index )
        self.last_frame = False
        self.flip_idx = 0
        self.lock = threading.Lock()
        self.fps = 60
        self.jump_frames = 0
        self.kick = True
        self.car_locations = [Queue(maxsize=4),Queue(maxsize=4), Queue(maxsize=4), Queue(maxsize=4), Queue(maxsize=4), Queue(maxsize=4)]
        #self.orange_kickoff_hierarchy = [[2048, -2560, 0], [-2048, -2560, 0], [256, -3840, 0], [-256, -3840, 0]]
    def get_output(self, packet):
        self.info.read_packet( packet )
        game = self.convert_packet_to_v3(packet)#Paul is a V3 Bot, a rework of preprocessing would be needed to re-optimize it
        self.frames = self.frames + 1
        self.renderer.begin_rendering()
        self.renderer.draw_string_2d(50, 50, 5, 5, str(time.time() - self.yeet_start), self.renderer.red())
        self.renderer.end_rendering()
        self.fps = self.frames / (time.time() - self.yeet_start)
        output = Bot.Process(self, game, packet, self.info, self.lock)
        controls = self.convert_output_to_v4(output)
        if (packet.game_info.is_kickoff_pause) :
            controls = self.yeet_kick_off.get_output( packet, self.frames / (time.time() - self.yeet_start) )
            self.last_frame = True

            return controls
        else:
            self.kick = True
            self.yeet_kick_off = KickoffAgent( 'eat ass', self.info.team, self.info.index )

        return controls
    def is_hot_reload_enabled( self ):
        return False





