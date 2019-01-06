import math
import random

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from RLUtilities.Maneuvers import Drive
from RLUtilities.GameInfo import GameInfo
from RLUtilities.Simulation import Car, Ball
from RLUtilities.LinearAlgebra import vec3, dot, clip, norm
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator
#from RLUtilities.controller_input import controller

class Agent(BaseAgent):

    def __init__(self, name, team, index):
        from RLUtilities.GameInfo import GameInfo
        self.info = GameInfo(index, team)
        self.controls = SimpleControllerState()
        self.action = None
        self.counter = 0
        self.index=index
    def dribble_steer(self):
        raise NotImplementedError
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.info.read_packet(packet)
        print('tester agent 27')
        if self.info.ball.pos[2]<=100:
            self.counter=0
            '''self.csign = random.choice([-1, 1])

            # this just initializes the car and ball
            # to different starting points each time
            c_position = Vector3(random.uniform(-1000, 1000),
                                 random.uniform(-4500, -4000),
                                 25)

            car_state = CarState(physics=Physics(
                location=c_position,
                velocity=Vector3(0, 0, 0),
                rotation=Rotator(0, 90, 0),
                angular_velocity=Vector3(0, 0, 0)
            ))

            self.bsign = random.choice([-1, 1])

            b_position = Vector3(c_position.x,c_position.y,
                                 150)

            b_velocity = Vector3(random.uniform(0,500),random.uniform(0,500),random.uniform(500,1000))

            ball_state = BallState(physics=Physics(
                location=b_position,
                velocity=Vector3(0,300,0),
                rotation=Rotator(0, 0, 0),
                angular_velocity=Vector3(0, 0, 0)
            ))

            self.set_game_state(GameState(
                ball=ball_state,
                cars={self.index: car_state})
            )'''
        prediction = Ball(self.info.ball)

        prediction.step(1/60)
        #if self.action == None or self.action.finished:
            #self.info.predict_ball(.0032)
            #target_pos = self.info.ball_predictions[len(self.info.ball_predictions-1)]
        target_pos=self.info.ball.pos
        #target_speed=2000
        '''if(math.sqrt(math.pow((target_pos[0]-self.info.my_car.pos[0]),2)+math.pow((target_pos[1]-self.info.my_car.pos[1]),2))<300):
            target_speed = math.sqrt(math.pow((target_pos[0]-self.info.my_car.pos[0]),2)+math.pow((target_pos[1]-self.info.my_car.pos[1]),2))*2'''
        target_speed=math.sqrt(math.pow(self.info.ball.vel[0],2)+math.pow(self.info.ball.vel[1],2))
        '''if(target_speed>1000):
            target_speed=1000'''
        self.action = Drive(self.info.my_car, target_pos, target_speed)
        self.counter+=1
        #self.counter = 0

        r = 200
        '''self.renderer.begin_rendering()
        purple = self.renderer.create_color(255, 230, 30, 230)

        self.renderer.draw_line_3d(self.action.target_pos - r * vec3(1, 0, 0),
                                   self.action.target_pos + r * vec3(1, 0, 0),
                                   purple)

        self.renderer.draw_line_3d(self.action.target_pos - r * vec3(0, 1, 0),
                                   self.action.target_pos + r * vec3(0, 1, 0),
                                   purple)

        self.renderer.draw_line_3d(self.action.target_pos - r * vec3(0, 0, 1),
                                   self.action.target_pos + r * vec3(0, 0, 1),
                                   purple)
        self.renderer.end_rendering()
'''




        self.counter += 1
        if (self.counter % 1) == 0:
            self.action.step(0.01666)
            self.controls = self.action.controls
        dist = math.sqrt(math.pow(self.info.ball.pos[0] - self.info.my_car.pos[0], 2) + math.pow(self.info.ball.pos[1] - self.info.my_car.pos[1], 2))
        if(dist<=30):
            self.controls.steer=0
        if(self.controls.steer!=0 and self.info.ball.pos[2]-self.info.my_car.pos[2]<=100):
            self.controls.steer=0
        return self.controls
