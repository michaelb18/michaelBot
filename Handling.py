from util import *
from PD import *
from RLUtilities.Maneuvers import AirDodge
'''
{
0  throttle:float; /// -1 for full reverse, 1 for full forward
1  steer:float; /// -1 for full left, 1 for full right
2  pitch:float; /// -1 for nose down, 1 for nose up
3  yaw:float; /// -1 for full left, 1 for full right
4  roll:float; /// -1 for roll left, 1 for roll right
5  jump:bool; /// true if you want to press the jump button
6  boost:bool; /// true if you want to press the boost button
7  handbrake:bool; /// true if you want to press the handbrake button
8  use_item:bool; /// true if you want to use a rumble item
}
'''
flip = [
    #0,1,2,3,4,5,   6,7
    [0,0,0,0,0,True,0,0],
    [0,0,-.5,0,0,True,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,-.5,0,0,True,0,0]
]

jump_shot = [
    #0,1,2,3,4,5,   6,7
    [0,0,0,0,0,True,0,0],
    [0,0,-.5,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,0,0,0,False,0,0],
    [0,0,-.5,0,0,True,0,0]
]
def controls(self):
    #All of Paul's targets and plans are already set into variables from Strategy.plan, so all we are doing here is feeding
    #that info into our PD controls

    self.powerslide = 0
   # if self.flip_idx == 0:
   #     global flip
   #     flip = [
   #         [0, 0, 0, 0, 0, True, 0, 0],
   #         [0, 0, -.5, 0, 0, True, 0, 0]
   #         #[0, 0, 0, 0, 0, True, 0, 0],
   #         #[0, 0, 0, 0, 0, True, 0, 0],
   #         #[0, 0, 0, 0, 0, True, 0, 0],
   #         #[0, 0, 0, 0, 0, True, 0, 0],
   #     ]
   #     for i in range(self.jump_frames):
   #         flip.append([0,0,0,0,0,True,0,0])
   #     flip.append([0,0,0,0,0,True,0,0])
   #     flip.append([0,0,0,0,0,False,0,0])
   #     flip.append([0,0,0,0,0,False,0,0])
   #     flip.append([0,0,-.5,0,0,True,0,0])
    #SlowDown(self)
    #Paul uses PD control
    self.steer = steer_from_angle(self.target_yaw_ang, self.yaw_vel, PI)

    self.throttle = throttle_velocity(self.player_local_vel[1], self.desired_speed, self.last_throttle)
    self.boost = self.throttle == 1 and boost_velocity(self.player_local_vel[1], self.desired_speed, self.last_boost) and self.player_loc[2] < 300
    if(self.flipping):
        action = flip[self.flip_idx]
        self.steer = action[1]
        self.pitch = action[2]
        self.yaw = action[3]
        self.roll = action[4]
        self.jump = action[5]
        self.boost = False
        self.flip_idx = self.flip_idx + 1
        if(self.flip_idx >= len(flip)):
            self.flipping = False
            self.flip_idx = 0
        #self.boost = False
    elif(self.jumping):
        self.pitch = 0
        self.yaw = 0;
        self.roll = 0;
    elif(not self.flipping):
        self.flip_idx = 0
def SlowDown(self):

    player_circle_local = a3([self.player_radius * sign(self.target_local_loc[0]), 0, 0])
    self.player_circle = world(player_circle_local, self.player_loc, self.player_rot)

    # if target is inside our turning radius
    if dist2d(self.player_circle, self.target_loc) < self.player_radius - 50:
        # slow down
        self.desired_speed = min(max(turning_speed(localized_circle_radius(
            *a2(self.target_local_loc))), 400), self.desired_speed)
        # if we're already too slow > powerslide
        if self.player_vel_mag < 400:
            self.powerslide = 1


def output(self):
    if(not self.on_ground):
        self.boost = False
    return [self.throttle, self.steer, self.pitch, self.yaw, self.roll, self.jump, self.boost, self.powerslide]
