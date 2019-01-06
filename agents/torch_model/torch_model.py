# MIT License
#
# Copyright (c) 2018 LHolten@Github Hytak#5125@Discord
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import sys
import math
import random
from rlbot.agents.base_agent import SimpleControllerState, BaseAgent, BOT_CONFIG_AGENT_HEADER
from rlbot.parsing.custom_config import ConfigHeader, ConfigObject
from RLUtilities.Maneuvers import Drive
path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, path)  # this is for first process imports
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator
from examples.levi.output_formatter import LeviOutputFormatter
from examples.levi.input_formatter import LeviInputFormatter
from agents.levitate.Aerial.agent import Agent as air
from RLUtilities.Maneuvers import Aerial
from agents.levitate.HalfFlip.agent import Agent as halfflip
from agents.tester.agent import Agent as dribbler
from agents.levitate.Dodge.agent import Agent as flip
from RLUtilities.Simulation import Ball
from collections import namedtuple
from agents.levitate.Aerial_Recovery.agent import Agent as air_recover
from agents.levitate.Defender import Agent as defense
Boost=namedtuple('Boost','dist x y')
class TorchModelAgent(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        sys.path.insert(0, path)  # this is for separate process imports
        import torch
        self.torch = torch
        self.empty_controller = SimpleControllerState()
        self.model_path = None
        self.model = None
        self.input_formatter = None
        self.output_formatter = None
        from RLUtilities.GameInfo import GameInfo
        from RLUtilities.GameInfo import BoostPad
        self.BoostPad=BoostPad
        self.move_steps=0
        self.info = GameInfo(index, team)
        self.air_agent = air('michaelBot', team, index)
        self.half_flip= halfflip('michaelBot',team ,index)
        self.the_dribble = dribbler('michaelBot',team,index)
        self.flip = flip('michaelBot',team,index)
        self.recover = air_recover('michaelBot', team , index)
        self.defense = defense('michaelBot',team,index)
        self.bot='model'
        self.counter=0
        self.controls=SimpleControllerState()
    def load_config(self, config_object_header: ConfigHeader):
        self.model_path = config_object_header.get('model_path')

    def initialize_agent(self):
        self.model = self.get_model()
        self.input_formatter = self.create_input_formatter()
        self.output_formatter = self.create_output_formatter()
        self.model.load_state_dict(self.torch.load(self.get_file_path()))

    def get_file_path(self):
        return os.path.join(path, self.model_path)

    def dist_to_nearest_boost(self):
        boost=self.info.boost_pads
        me=self.info.my_car.pos
        minDist=math.inf
        yeet=Vector3(self.info.boost_pads[0].pos[0],self.info.boost_pads[0].pos[1],self.info.boost_pads[0].pos[2])
        for b in boost:
            dist=math.sqrt(math.pow(b.pos[0]-me[0],2)+math.pow(b.pos[1]-me[1],2))
            if(dist<minDist and b.is_active):
                minDist=dist
                yeet=b
        return yeet,minDist
    def get_angle_to(self,pos,packet):
        me=Vector2(self.info.my_car.pos[0],self.info.my_car.pos[1])
        pos=Vector2(pos[0],pos[1])
        vec=pos-me
        angle_to = math.atan2(vec.y, vec.x) * (180 / math.pi)

        car_angle = get_car_facing_vector(packet.game_cars[self.index])
        car_heading = math.atan2(car_angle.y, car_angle.x) * (180 / math.pi)
        return abs(angle_to-car_heading)
    def get_output(self, packet):
        """
        Predicts an output given the input
        :param packet: The game_tick_packet
        :return:
        """
        self.info.read_packet(packet)
        prediction = Ball(self.info.ball)
        prediction.step(10/60)
        ball_p=Vector3(self.info.ball.pos[0],self.info.ball.pos[1],self.info.ball.pos[2])
        ball_p_f=Vector3(prediction.pos[0],prediction.pos[1],prediction.pos[2])
        me_p=Vector3(self.info.my_car.pos[0],self.info.my_car.pos[1],self.info.my_car.pos[2])
        vec=ball_p_f-me_p
        angle_to=math.atan2(vec.y,vec.x)*(180/math.pi)

        car_angle=get_car_facing_vector(packet.game_cars[self.index])
        car_heading=math.atan2(car_angle.y,car_angle.x)*(180/math.pi)

        dist=math.sqrt(math.pow(ball_p.x-me_p.x,2)+math.pow(ball_p.y-me_p.y,2))
        dist3 = math.sqrt(math.pow(ball_p.x - me_p.x, 2) + math.pow(ball_p.y - me_p.y, 2) + math.pow(ball_p.z - me_p.z, 2))
        nearest_boost=self.dist_to_nearest_boost()

        angle_to=abs(angle_to-car_heading)

        boost_angle=self.get_angle_to(nearest_boost[0].pos,packet)
        self.renderer.begin_rendering()
        self.renderer.draw_line_2d(0,0,vec.x,vec.y,self.renderer.black())
        self.renderer.draw_string_2d(100,100,5,5,'Round Active: '+str(packet.game_info.is_round_active),self.renderer.red())
        self.renderer.draw_string_2d(100, 175, 5, 5, 'Bot: '+self.bot,
                                     self.renderer.red())
        self.renderer.draw_string_2d(100, 250, 5, 5, 'Angle to Ball: ' + str(angle_to),
                                     self.renderer.red())
        self.renderer.draw_string_2d(100, 325, 5, 5, 'Angle to Nearest Boost: ' + str(boost_angle),
                                     self.renderer.red())
        self.renderer.draw_string_2d(100, 400, 5, 5, 'On Ground: ' + str(self.info.my_car.on_ground),
                                     self.renderer.red())
        self.renderer.end_rendering()
        if nearest_boost[1]<1000 and boost_angle<60:
            self.bot='boost_retrieval'
            target_pos=nearest_boost[0].pos

            target_speed = 2500
            self.action = Drive(self.info.my_car, target_pos, target_speed)
            self.counter += 1
            # self.counter = 0
            if (self.counter % 10) == 0:
                self.action.step(0.01666)
                self.controls = self.action.controls
            return self.controls
        if(self.info.my_car.boost>100):
            print('[W - torchmodel.py 113] Invalid Boost!')
        if(self.move_steps==200):
            self.move_steps=0
        '''if((self.info.team == 1 and self.info.ball.vel[1]>0) and (self.info.team == 0 and self.info.ball.vel[1]<0) and math.sqrt(math.pow(self.info.ball.pos[0]-self.info.my_goal.pos[0],2)+math.pow(self.info.ball.pos[1]-self.info.my_goal.pos[1],2))<math.sqrt(math.pow(self.info.ball.pos[0]-self.info.their_goal.pos[0],2)+math.pow(self.info.ball.pos[1]-self.info.their_goal.pos[1],2))):
            self.bot='defense'
            return self.the_d.get_output(packet)'''
        if(dist3<=300):
            self.bot='flip'
            return self.flip.get_output(packet)
        if((angle_to>135 and angle_to<225) or (self.half_flip.action is not None and not self.half_flip.action.finished)):
            self.move_steps+=1
            self.bot='half'
            return self.half_flip.get_output(packet)
        if (self.info.ball.pos[2] > 500 and self.info.my_car.boost>=25 and (angle_to<60 or angle_to>300) or (self.air_agent.action is not None and not self.air_agent.action.finished)):
            self.move_steps+=1
            self.bot='air'
            return self.air_agent.get_output(packet)
        if(not self.info.my_car.on_ground):
            self.bot='recover'
            self.recover.get_output(packet)
        self.move_steps=0
        if not packet.game_info.is_round_active:
            return self.empty_controller
        if packet.game_cars[self.index].is_demolished:
            return self.empty_controller

        arr = self.input_formatter.create_input_array([packet], batch_size=1)

        assert (arr[0].shape == (1, 3, 9))
        assert (arr[1].shape == (1, 5))
        self.bot='model'
        output = self.advanced_step(arr)
        return self.output_formatter.format_model_output(output, [packet], batch_size=1)[0]
        #return SimpleControllerState()
    def create_input_formatter(self):
        return LeviInputFormatter(self.team, self.index)

    def create_output_formatter(self):
        return LeviOutputFormatter(self.index)

    def get_model(self):
        from examples.levi.torch_model import SymmetricModel
        return SymmetricModel()

    def advanced_step(self, arr):
        arr = [self.torch.from_numpy(x).float() for x in arr]

        with self.torch.no_grad():
            output = self.model.forward(*arr)
        return output

    @staticmethod
    def create_agent_configurations(config: ConfigObject):
        super(TorchModelAgent, TorchModelAgent).create_agent_configurations(config)
        params = config.get_header(BOT_CONFIG_AGENT_HEADER)
        params.add_value('model_path', str, default=os.path.join('models', 'cool_atba.mdl'),
                         description='Path to the model file')
class Vector2:
    def __init__(self, x=0, y=0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, val):
        return Vector2(self.x + val.x, self.y + val.y)

    def __sub__(self, val):
        return Vector2(self.x - val.x, self.y - val.y)

    def correction_to(self, ideal):
        # The in-game axes are left handed, so use -x
        current_in_radians = math.atan2(self.y, -self.x)
        ideal_in_radians = math.atan2(ideal.y, -ideal.x)

        correction = ideal_in_radians - current_in_radians

        # Make sure we go the 'short way'
        if abs(correction) > math.pi:
            if correction < 0:
                correction += 2 * math.pi
            else:
                correction -= 2 * math.pi

        return correction


def get_car_facing_vector(car):
    pitch = float(car.physics.rotation.pitch)
    yaw = float(car.physics.rotation.yaw)

    facing_x = math.cos(pitch) * math.cos(yaw)
    facing_y = math.cos(pitch) * math.sin(yaw)

    return Vector2(facing_x, facing_y)
