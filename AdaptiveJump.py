from util import *
class Jump:
    def __init__(self):
        self.jump_time = 2

    def evaluate( self, ball_pos, enemy_pos, my_pos, goal, their_goal, pos1, pos2 ):
        if(dist(pos1, their_goal) > dist(their_goal, pos2)):
            print("Jump Successful - no time change")
        else:
            if(dist(ball_pos,goal) < dist(my_pos,goal)):
                #self.jump_time += .05
                print("Jump Failed - late. Time is now" + str(self.jump_time))
            else:
                #self.jump_time -= .05
                print("Jump Failed - early. Time is now" + str(self.jump_time))