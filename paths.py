from util import *
from classes import *
import scipy.interpolate
from PD import *
""" Lines """
#These are for setting up Dynamic Lines, which are able to adapt to the current situation
#instead of having fixed start and end points
def ball_to_goal_update(line, agent, index):
    speed = mag(agent.player_vel)
    #line.events[0].speed = speed
    distance = dist3d(agent.goal, agent.ball_loc)
    line.start = set_dist(agent.goal, agent.ball_loc, distance + (turning_radius(speed)*2))
    line.end = set_dist(agent.goal, agent.ball_loc, distance + 150)
    a = -650
    b = agent.ball_vel[2]
    c = agent.ball_loc[2]
    t = (-b - math.sqrt(b**2 - 4 * a * c)) / (2 * a)
    ball = agent.ball_loc + agent.ball_vel * t

    '''if(agent.ball_loc[2] > 150):
        aim = ball - agent.goal
        aim = ball - agent.goal
        pos = ball
        d = dist( agent.player_loc, ball )
        line.events[0].speed = d / t
        line.end = (ball)
    else:
        line.events[0].speed = 2300'''
    print('ball_to_goal')


def hit_update(line, agent, index):
    speed = 2300
    a = -650
    b = agent.ball_vel[2]
    c = agent.ball_loc[2]
    t = (-b - math.sqrt(b**2 - 4 * a * c)) / (2 * a)
    ball = agent.ball_loc + agent.ball_vel * t
    aim = ball - agent.goal
    pos = agent.goal + .25 * normalize( aim )
    if(agent.ball_loc[2] > 150):

        d = dist( agent.player_loc, ball )
        line.events[0].speed = d / t
        line.end = (agent.goal)
        line.start = (ball)
    else:
        line.events[0].speed = 2300
        line.start = (agent.ball_loc)
        line.end = (agent.goal)

    print('hit')

def car_to_next_update(line,agent,index):
    line.start = (agent.player_loc)
    line.end = (agent.path.lines[index+1].start)
    line.events[0].speed = 2300

def shot_update(line, agent, index):
    post_1 = np.array( [agent.goal[0] + 893, agent.goal[1], 0] )
    post_2 = np.array( [agent.goal[0] - 893, agent.goal[1], 0] )
    v1 = flatten( agent.ball_loc - post_1 )
    v2 = flatten( agent.ball_loc - post_2 )
    p1 = flatten( agent.ball_loc )
    p2 = v1 + p1
    p3 = v2 + p1
    triangle = Triangle( p1, flatten( confine_to_field( p2 ) ),
                         flatten( confine_to_field( p3 ) ) )
    if (triangle.isInside( agent.player_loc )) :
        a = -650
        b = agent.ball_vel[2]
        c = agent.ball_loc[2]
        t = (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
        ball = agent.ball_loc + agent.ball_vel * t
        aim = ball - agent.goal
        pos = agent.goal + .25 * normalize( aim )
        if (agent.ball_loc[2]>150) :

            d = dist( agent.player_loc, ball )
            line.events[0].speed = d / t
            line.end = (agent.goal)
            line.start = (ball)
        else :
            line.events[0].speed = 2300
            line.start = (agent.ball_loc)
            line.end = (agent.goal)
    else :
        line.start = (agent.player_loc)
        end = agent.ball_loc - agent.player_loc
        line.end = (triangle.centroid)
        line.events[0].speed = 2300
v_init = 69
def to_centroid(l, agent, index):
    global v_init
    if(v_init == 69):
        print('v init')
        v_init = mag(agent.player_vel)
    post_1 = np.array([agent.goal[0] + 893, agent.goal[1], 0])
    post_2 = np.array([agent.goal[0] - 893, agent.goal[1], 0])
    my_vel = mag(agent.player_vel)
    v_avg = abs(my_vel - mag(agent.ball_vel))/2
    v1 = flatten( agent.ball_loc - post_1 )
    v2 = flatten( agent.ball_loc - post_2 )
    p1 = flatten( agent.ball_loc )
    p2 = v1 + p1
    p3 = v2 + p1
    triangle = Triangle( p1, flatten( confine_to_field( p2 ) ),
                             flatten( confine_to_field( p3 ) ) )
    c_dist = dist(triangle.centroid , agent.ball_loc)
    speed = dist(triangle.centroid, agent.ball_loc) / 2
    ball = agent.ball_loc + agent.ball_vel * 2
    v1 = flatten( ball - post_1 )
    v2 = flatten( ball - post_2 )
    p1 = flatten( ball )
    p2 = v1 + p1
    p3 = v2 + p1
    #triangle = Triangle( p1, flatten( confine_to_field( p2 ) ),
    #                     flatten( confine_to_field( p3 ) ) )
    l.start = (agent.ball_loc - normalize(agent.goal - ball) * 2000)
    l.end   = (agent.ball_loc)
    l.events[0].speed = 2400
def clear_update(line, agent, index):
    print('clearing')
    post_1 = np.array( [agent.my_goal[0] + 893, agent.my_goal[1], 0] )
    post_2 = np.array( [agent.my_goal[0] - 893, agent.my_goal[1], 0] )
    v1 = flatten( agent.ball_loc - post_1 )
    v2 = flatten( agent.ball_loc - post_2 )
    p1 = flatten( agent.ball_loc )
    p2 = v1 + p1
    p3 = v2 + p1
    triangle = Triangle( p1, flatten( confine_to_field( p2 ) ),
                         flatten( confine_to_field( p3 ) ) )
    if (not triangle.isInside( agent.player_loc )) :
        line.start = (agent.player_loc)
        line.end = (agent.ball_loc)
        line.events[0].speed = 2300
    else :
        line.start = (np.array([agent.ball_loc[0] + 893 * sign(agent.player_loc[0]), agent.ball_loc[1], agent.ball_loc[2]]))
        line.end = (agent.ball_loc)
        line.events[0].speed = 2300
def teamplay(line, agent, index):
    print('teamplay')
    def find_a_boost_pad(loc):
        bRet = agent.boost_pads[0]
        for b in agent.boost_pads:
            if(turn_radius(mag(agent.player_vel)) < dist2d(np.array([b.pos[0], b.pos[1], b.pos[2]]), agent.player_loc)):
                if(dist(loc, np.array([b.pos[0], b.pos[1], b.pos[2]])) < dist(loc, np.array([bRet.pos[0], bRet.pos[1], bRet.pos[2]]))):
                    bRet = b
        return np.array([bRet.pos[0], bRet.pos[1], 0])
    line.start = (agent.player_loc)

    a = -650
    b = agent.ball_vel[2]

    c = agent.ball_loc[2]
    t = (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
    if(t < 0):
        (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
    ball_loc= (np.array([agent.ball_loc[0] + agent.ball_vel[0] * t,
                         agent.ball_loc[1] + agent.ball_vel[1] * t,
                         0]))
    line.end = (find_a_boost_pad(ball_loc))
    print(line.end)
    line.events[0].speed = 1410
    if(dist(agent.player_loc, agent.ball_loc)/1410 > 5):
        line.events[0].flip = True

import threading
def catch_update(line,agent,index):
    line.start = (agent.player_loc)

    a = -650
    b = agent.ball_vel[2]

    c = agent.ball_loc[2]
    t = (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
    if(t < 0):
        (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
    line.end = (np.array([agent.ball_loc[0] + agent.ball_vel[0] * t,
                         agent.ball_loc[1] + agent.ball_vel[1] * t,
                         0]))

    ball_loc = line.end
    ball_goal = line.end - agent.goal
    save =  normalize(agent.my_goal-agent.ball_loc)
    #max_d =
    #x = np.linspace(0, dist2d(np.array([3072, -4096, 73]), np.array([-3072, 4096, 73])), 50)
    #y = (np.linspace(min_d , max_d, 50) ** 2) / 90
    #interpoland = scipy.interpolate.interp1d(x, y, kind = 'linear', fill_value='extrapolate')
    #offset = interpoland( max(dist2d( agent.ball_loc, agent.my_goal ), dist2d( agent.ball_loc, agent.goal)) )
    t_goal = (agent.my_goal[1] - agent.ball_loc[1])/agent.ball_vel[1]

    def i_am_the_one( self ) :
        for c in self.teammates :
            team_t = dist( np.array( [c.pos[0], c.pos[1], c.pos[2]] ), self.ball_loc ) / mag(
                np.array( [c.vel[0], c.vel[1], c.vel[2]] ) )
            player_t = dist( self.player_loc, self.ball_loc ) / mag( self.player_vel )
            if (team_t<player_t) :
                return False
        return True
    def find_a_boost_pad(loc):
        bRet = agent.boost_pads[0]
        for b in agent.boost_pads:
            if(b.is_active and turn_radius(mag(agent.player_vel)) < dist2d(np.array([b.pos[0], b.pos[1], b.pos[2]]), agent.player_loc)):
                if(dist(loc, np.array([b.pos[0], b.pos[1], b.pos[2]])) < dist(loc, np.array([bRet.pos[0], bRet.pos[1], bRet.pos[2]]))):
                    bRet = b
        return np.array([bRet.pos[0], bRet.pos[1], 0])
    def teammate_advantageous():
        '''dists = []
        for team in agent.teammates:
            dists.append(dist2d(team.pos, agent.ball_loc + agent.ball_vel * t))
        for d in dists:
            if dist2d(agent.player_loc, agent.ball_loc + agent.ball_vel * t) < .9 * d:
                print('BUT HE IN ASS POSITION AKDJHAK:SFJ A:LJSF A:LJSF A:LJSF :JSAF AS:jf ')
                return False
        return True'''
        if(dist(agent.player_loc, agent.ball_loc) < 100):
            force = agent.player_loc - agent.ball_loc
            t_owngoal = (agent.my_goal[1] - agent.ball_loc[1]) / force[1]
            if(t_owngoal > 0 and t_owngoal < 30):
               for team in agent.teammates:
                   t_team = dist(np.array([team.pos[0], team.pos[1], team.pos[2]]), agent.ball_loc)/mag(np.array([team.vel[0], team.vel[1], team.vel[2]]))
                   if(t_team < 5):
                       return True
        return False

    if(t_goal < 30 and t_goal > 0 and abs((agent.ball_vel * t_goal + agent.ball_loc)[0]) < 2000):
        #if (dist2d( agent.ball_loc, agent.my_goal )<2000) :
        line.end = line.end + 90 * normalize( save )
        #else :
            #line.end = line.end + 45 * normalize( save )
    else:
        if(dist2d(agent.ball_loc, agent.goal) < 2000):
            line.end =  line.end + 100 * normalize(ball_goal)
        else:
            line.end = line.end + 45 * normalize(ball_goal)#35 - 80
    #print(offset)
    #line.end = line.end + 50 * normalize( ball_goal )  # 35 - 80

    #if(teammate_advantageous()):
    #    line.end = find_a_boost_pad()
    #else:
    line.events[0].speed = dist(line.end, agent.player_loc) / t
    '''if (dist2d( agent.player_loc, agent.ball_loc )<93.5) :
        enemy_vel = normalize(flatten(agent.enemy_vel))
        enemy_loc = flatten(agent.enemy_loc)
        ball_loc = flatten(agent.ball_loc)
        ball_vel = normalize(flatten(agent.ball_vel))
        l0 = ball_loc - enemy_loc
        t2 = np.dot(l0, enemy_vel)
        line.events[0].jump = False
        if(t2 >= 0):
            d = math.sqrt( np.dot(l0, l0) - t2 ** 2 )
            if(d >= 0 and d <= 92.75):
                t3 = math.sqrt( 92.75 ** 2 - d ** 2)

                t0 = t2 - t3
                t1 = t2 + t3
                if(t0 > 0 and t0 < 1500):
                        line.events[0].jump = True
                elif(t1 > 0 and t1 < 1500):
                        line.events[0].jump = True'''
    line.events[0].jump = False
    line.events[0].flip = False
    print('-------------------------------------')
    for o in agent.opponents:
        tj = 0
        while tj <= 2:
            enemy_loc = np.array([o.pos[0], o.pos[1], o.pos[2]]) + agent.enemy_loc * tj #instead of this tjump stupidity, feed in the path generated by get_drive_path, but only if the queue is full
            #enemy_loc = path(tj)
            ball_loc = agent.ball_loc + agent.ball_vel * tj
            if(dist2d(enemy_loc, ball_loc) < 150):
                print(ball_loc[2] - agent.player_loc[2])
                if(dist2d(agent.ball_loc,agent.player_loc) <= 125):
                    print("THIS gUn DOES NO DAMAGE")
                    line.events[0].jump = True
                elif(agent.ball_loc[2] < 120):
                    line.events[0].flip = True
                '''elif(dist2d( agent.player_loc, ball_loc )<125):
                    a_prime = 1458.333374 + 650
                    b_prime = 291.667 +agent.ball_vel[2]
                    c_prime =17.01 - ball_loc[2]
                    t = (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
                    if (t<0) :
                        (-b - math.sqrt( b**2 - 4 * a * c )) / (2 * a)
                    line.end = (np.array( [agent.ball_loc[0] + agent.ball_vel[0] * t,
                                           agent.ball_loc[1] + agent.ball_vel[1] * t,
                                           0] ))
                    agent.jump_frames = int(agent.fps * t)
                    line.events[0].flip = True
                if(not agent.jumping):
                    thread = threading.Thread(target=eval_jump, args=(agent, o,))
                    thread.start()'''
                agent.jumping = True
            tj = tj + .1
        #elif( inField(agent.line.end) and agent.player_loc[2] < 100 and dist2d( agent.player_loc, agent.line.end )>agent.player_radius + 1500 and agent.desired_speed > 2300 and abs(steer_from_angle(agent.target_yaw_ang, agent.yaw_vel, PI)) < .05 and mag(agent.player_vel) >= 1300):
        #    line.events[0].flip = True
    agent.jumping = line.events[0].jump
    if(not i_am_the_one(agent)):
        print('we teaming')
        line.end = (find_a_boost_pad(line.end))
        line.events[0].speed = 1410
        return





def shot2_update(line, agent, index):
    line.start = agent.ball_loc
    line.end = agent.goal
    line.events[0].speed = 2300
#each line is still a line object, but the update function of the object is ammended
ball_to_goal = line()
ball_to_goal.update = ball_to_goal_update

car_to_next = line()
car_to_next.update = car_to_next_update

centroid = line()
centroid.update = to_centroid

catch = line()
catch.update = catch_update

shot = line()
shot.update = shot2_update
teamp = line()
teamp.update = teamplay
hit = line()
hit.update = hit_update
clear = line()
clear.update = clear_update
""" Paths """
#A path is a collection of lines, and called a Dynamic Path if one or more lines are Dynamic
class pathTypes():
    def __init__(self):
        self.basicShot = path([centroid])
        self.basicShot2 = path()
        self.catch = path([catch])
        self.clear = path([clear])
        self.teamp = path([teamp])
pathTypes = pathTypes()
