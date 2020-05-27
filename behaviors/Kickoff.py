
#import sys
#from os.path import dirname
from RLUtilities.Maneuvers import Drive, AirDodge
from RLUtilities.GameInfo import GameInfo
from RLUtilities.LinearAlgebra import vec3
import time
#sys.path.append(dirname(dirname(dirname(__file__))))

from util import *

class Flip():
    def __init__( self, info:GameInfo, fps ):

        self.action = Drive(info.my_car,info.ball.pos,2400)
        self.rate = .75#s
        self.time = time.time()
        self.fps = fps
        self.flip = AirDodge(info.my_car,duration=.1,target=info.ball.pos)
        self.done = False

        self.do_flip = False
    def execute( self, read_info: GameInfo, fps ):
        self.fps = fps
        if(not self.done):
            if(self.do_flip):
                self.flip.step(1.0/self.fps)
                self.do_flip = not self.flip.finished
                return YeetControls.from_simple_controller(self.flip.controls)
            else:
                self.action.step(1.0/self.fps)
                controls = self.action.controls
                controls.boost = True
            if (time.time() - self.time>self.rate) :
                self.time = time.time()
                self.do_flip = True
                self.flip = AirDodge( read_info.my_car, duration=.1, target=read_info.ball.pos )

            return YeetControls.from_simple_controller(controls)
        return YeetControls.from_simple_controller(SimpleControllerState())
    def cancel( self ):
        self.done = True


class Kickoff():
    def __init__( self, info:GameInfo, fps ):

        self.info = info
        if info.team == 1:
            team = 1
        else:
            team = -1
        self.kickoff_hierarchy = [np.multiply(np.array( [-2048, -2560, 0] ), team), np.multiply(np.array( [2048, -2560, 0] ), team ),
                                  np.multiply( np.array( [-256, -3840, 0] ), team ), np.multiply(np.array( [256, -3840, 0] ), team) ]
        self.action = Drive( info.my_car)
        self.action.target_pos = info.ball.pos
        self.action.target_speed = 2400
        t_factor = 1
        self.boost = True
        if(len(self.info.teammates) > 1 and self.check_kickoff() and not self.is_closest()):
            self.action = Drive(info.my_car)#, np.array([0, 450 * team, 0]), 2400)
            self.action.target_pos = vec3(0, 450 * team, 0)#np.array([0, 450 * team, 0])
            self.action.target_speed = 1410
            t_factor = 1000000
            self.boost = False
        car_ball_distance = dist(info.my_car.pos, info.ball.pos)
        t = 1.5 * t_factor
        if(abs(car_ball_distance - 3848.523872863465) < 1):
            t = 2.05 * t_factor
        elif(abs(car_ball_distance - 4608) < 1):
            t = 2.15 * t_factor
        self.rate = t#s
        print(t)
        self.time = time.time()
        self.fps = fps
        self.flip = AirDodge(info.my_car,duration=.1,target=info.ball.pos)
        self.done = False
        self.do_flip = False

    def check_kickoff( self ):
        def find_pos(arr:np.ndarray):
            pos = 0
            for p in self.kickoff_hierarchy:
                print(p, arr)
                if(abs(p[0] - arr[0]) < 100 and abs(p[1] - arr[1]) < 100):
                    return pos
                pos = pos + 1
            return -1
        pos = find_pos(np.array([self.info.my_car.pos[0],self.info.my_car.pos[1],self.info.my_car.pos[2]]))
        print(pos)
        for team in self.info.teammates:
            if(find_pos(np.array([team.pos[0], team.pos[1], team.pos[2]])) < pos):
               return False
        return True
    def is_closest( self ):
        for team in self.info.teammates:
            if(dist2d(team.pos, self.info.ball.pos) < dist2d(self.info.my_car.pos, self.info.ball.pos)):
                return False

        return True
    def execute( self, read_info: GameInfo, fps ):
        self.fps = fps
        if(not self.done):
            if(self.do_flip):
                self.flip.step(1.0/self.fps)
                self.do_flip = not self.flip.finished
                return YeetControls.from_simple_controller(self.flip.controls)
            else:
                self.action.step(1.0/self.fps)
                controls = self.action.controls
                controls.boost = self.boost
            if (time.time() - self.time>self.rate and self.boost) :
                self.time = time.time()
                self.do_flip = self.boost
                self.flip = AirDodge( read_info.my_car, duration=.1, target=read_info.ball.pos )
            if(not self.do_flip):
                controls.jump = False
            return YeetControls.from_simple_controller(controls)
        return YeetControls.from_simple_controller(SimpleControllerState())
    def cancel( self ):
        self.done = True