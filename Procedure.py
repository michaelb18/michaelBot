from util import *
from RLUtilities.Maneuvers import AirDodge
from RLUtilities.GameInfo import GameInfo
def rlu_pp(self, info):
    self.teammates = info.teammates
    self.opponents = info.opponents
    for s in self.teammates:
        if(not self.car_locations[s.id].full()):
            self.car_locations[s.id].put_nowait(np.array( [s.pos[0],s.pos[1], s.pos[2]] ))
        else:
            self.car_locations[s.id].get_nowait()
            self.car_locations[s.id].put_nowait( np.array( [s.pos[0], s.pos[1], s.pos[2]] ) )
    for s in self.opponents:
        if (not self.car_locations[s.id].full()) :
            self.car_locations[s.id].put_nowait(np.array( [s.pos[0],s.pos[1], s.pos[2]] ))
        else:
            self.car_locations[s.id].get_nowait()
            self.car_locations[s.id].put_nowait( np.array( [s.pos[0], s.pos[1], s.pos[2]] ) )

    #self.forward = np.array([info.my_car.forward()[0], info.my_car.forward()[1], info.my_car.forward()[2]])
    self.boost_pads = list(set(info.boost_pads)|set(info.small_boost_pads))
def pre_process(self, game, packet, lock): #Turns gamepacket into more useful values
    self.game = game

    self.player = game.gamecars[self.index]
    self.on_ground = packet.game_cars[self.index].has_wheel_contact
    self.jumping = False
    enemy_index = 1
    if(enemy_index == self.index):
        enemy_index = 0
    self.ball = game.gameball
    self.enemy = game.gamecars[enemy_index]

    # converting vector3 and rotator classes into numpy arrays
    self.player_loc = a3(self.player.Location)
    self.player_vel = a3(self.player.Velocity)
    self.player_rot = a3(self.player.Rotation)
    self.player_angvel = a3(self.player.AngularVelocity)

    self.enemy_loc = a3(self.enemy.Location)
    self.enemy_vel = a3(self.enemy.Velocity)

    self.ball_loc = a3(self.ball.Location)
    self.ball_vel = a3(self.ball.Velocity)

    if not hasattr(self, 'counter'):

        self.counter = 0
        if(self.counter == 0):
            self.flipping = False
        self.throttle = self.steer = self.pitch = self.yaw = self.roll = self.jump = self.boost = self.powerslide = 0

        self.target_loc = self.player_loc
        self.desired_speed = MAX_CAR_SPEED

        self.throttle_type = "point"

        feedback(self)

    # converting the veloctites to local coordinates
    self.pitch_vel, self.roll_vel, self.yaw_vel = local(self.player_angvel, ZEROS3, self.player_rot)
    self.player_local_vel = local(self.player_vel, ZEROS3, self.player_rot)
    self.player_vel_mag, self.player_vel_yaw_ang, self.player_vel_pitch_ang = spherical(*self.player_local_vel)

    # approximate turning radius
    self.player_radius = turning_radius(dist2d(self.player_local_vel))
    self.is_kickoff = is_kickoff(self.player_loc, self.player_vel)
    self.color = -sign(self.player.Team)
    self.goal = a3([0, FIELD_LENGTH / 2 * self.color, 0])
    self.my_goal = a3([0, FIELD_LENGTH / 2 * sign(self.player.Team), 0])

    self.boost_level = packet.game_cars[self.index].boost
def feedback(self):

    self.last_throttle, self.last_steer = self.throttle, self.steer
    self.last_pitch, self.last_yaw, self.last_roll = self.pitch, self.yaw, self.roll
    self.last_jump, self.last_boost, self.last_powersilde = self.jump, self.boost, self.powerslide
    self.last_flip = self.flipping
    self.counter += 1

def is_kickoff(agent_loc, agent_vel):
    if(abs(flatten(agent_loc)[0] - 2048) < 5 and abs(flatten(agent_loc)[1] - 2560) < 5):
        if(mag(agent_vel) == 0):
            return True
    if (abs( flatten( agent_loc )[0] - 256 )<5 and abs( flatten( agent_loc )[1] - 3840 )<5) :
        if(mag(agent_vel) == 0):
            return True
    if (abs( flatten( agent_loc )[0]  )<5 and abs( flatten( agent_loc )[1] - 4608 )<5) :
        if(mag(agent_vel) == 0):
            return True
    return False
def finish(self):
        
    #Some paths are dynamic, meaning they change depending on the game conditions
    #This is where the path is updated from    
    self.path.update(self) 
    self.path.lines[0].events[0].jump = False
    self.path.lines[0].events[0].flip = False
    #An old method for deleting a user-drawn path after it has been completed 
    '''
    if len(self.path.lines) != 0 and hasattr(self, "line"):

        if dist3d(self.player_loc, self.line.start) < 99:
            self.line.started = True
        
        
        if self.line.started and dist2d(self.player_loc, self.line.end) < 99:
            self.line.finished = True
            self.path.delete(self)
    '''
