import math
import numpy as np
import pickle
from RLUtilities.LinearAlgebra import vec3
from rlbot.agents.base_agent import SimpleControllerState
U180 = 32768
U90 = U180 / 2
PI = math.pi

ZEROS3 = np.zeros(3)
ZEROS2 = np.zeros(2)
ZEROS3.flags.writeable = False
ZEROS2.flags.writeable = False

GOAL_WIDTH = 1900
FIELD_LENGTH = 10280
FIELD_WIDTH = 8240

BALL_RADIUS = 93

MIN_TURN_RADIUS = 160
MAX_CAR_SPEED = 2300
BRAKES_RATE = 3600  # deceleration in uu/s²


''' utility functions sorted in alphabetical order '''


# converts a vector, rotator or a normal list to a numpy array
def a2(v):
    try:
        a = np.array([v[0], v[1]])
    except TypeError:
        a = np.array([v.X, v.Y])
    return a


def a3(V):
    try:
        a = np.array([V.X, V.Y, V.Z])
    except AttributeError:
        try:
            a = np.array([V.Pitch, V.Yaw, V.Roll])
        except AttributeError:
            a = np.array([V[0], V[1], V[2]])
    return a


def ang_dif(a1, a2, pi=PI):  # difference between two angles
    return abs(Range180(a1 - a2, pi))


def angle(a, b):  # Returns angle between 2 objects in radians
    a, b = a2(a), a2(b)
    return math.atan2(a[1] - b[1], a[0] - b[0])


def dist2d(a, b=ZEROS2):  # distance/vector length in 2d
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def dist3d(a, b=ZEROS3):  # distance/vector length in 3d
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def line_intersect(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        div = 1
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def line_projection(line_point1, line_point2, point):
    x1, y1 = line_point1[0], line_point1[1]
    x2, y2 = line_point2[0], line_point2[1]
    x3, y3 = point[0], point[1]

    mag = (y2 - y1) ** 2 + (x2 - x1) ** 2
    if mag != 0:
        k = ((y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1)) / mag
    else:
        k = 1

    x4 = x3 - k * (y2 - y1)
    y4 = y3 + k * (x2 - x1)
    return x4, y4


# transforms global/world into local coordinates, see rlbot start-to-finish tutorials for a better method :)
def local(tL, oL, oR, Urot=True):
    L = tL - oL
    if Urot:
        pitch = oR[0] * PI / U180
        yaw = Range180(oR[1] - U90, U180) * PI / U180
        roll = oR[2] * PI / U180
        R = [-pitch, -yaw, -roll]
    else:
        R = -oR
    x, y = rotate2D(L[0], L[1], R[1])
    y, z = rotate2D(y, L[2], R[0])
    x, z = rotate2D(x, z, R[2])
    return np.array([x, y, z])


def local_2d(tL, oL, ang):
    L = a2(tL) - a2(oL)
    return a2(rotate2D(*L, ang))


def localized_circle_radius(x, y):
    if x != 0:
        return (x**2 + y**2) / abs(2 * x)
    else:
        return y**2


# returns the point between 2 other points, can be skewed towards the second point by a factor
def midpoint(a, b, factor=1):
    return (a[0] + (factor * b[0])) / (1 + factor), (a[1] + (factor * b[1])) / (1 + factor)


# sets the vector length to 1
def normalize(A):
    mag = np.linalg.norm(A)
    if mag == 0:
        mag = 1e-9
    return A / mag


# limits a value to a max and optionaly a min, eg: Range(-900, 500) = -500
def Range(value, max_value, min_value=0):
    if abs(value) > max_value:
        value = math.copysign(max_value, value)
    if abs(value) < min_value:
        value = math.copysign(min_value, value)
    return value


def Range180(a, pi=PI):  # fixes any angle ammount to [-pi, pi] range, eg: Range180(270,180) = -90
    if abs(a) >= 2 * pi:
        a -= abs(a) // (2 * pi) * 2 * pi * sign(a)
    if abs(a) > pi:
        a -= 2 * pi * sign(a)
    return a


def Range360(a, pi=PI):  # fixes any angle ammount to [0, 360] range
    return a - (a // (2 * pi)) * 2 * pi


def readFile(file):  # reads an object from a file
    data = pickle.load(open(file, 'rb'))
    return data


def relative_angle(origin_point, point_A, point_B):  # relative angle between two points from an origin
    return Range180(angle(point_A, origin_point) - angle(point_B, origin_point), PI)


# rotates a 2d vector by an angle
def rotate2D(x, y, ang):
    x2 = x * math.cos(ang) - y * math.sin(ang)
    y2 = y * math.cos(ang) + x * math.sin(ang)
    return x2, y2


# returns a point a certain distance from a towards b
def set_dist(a, b, dist=1):
    c = a3(b) - a3(a)
    c = normalize(c) * dist + a
    return c


# returns a point a certain distance from a towards b using 2d coordinates
def set_dist2d(a, b, dist=1):
    c = a2(b) - a2(a)
    c = normalize(c) * dist + a2(a)
    return c


def sign(x):
    if x > 0:  # > instead of >= so that sign(False) returns -1
        return 1
    else:
        return -1


def spherical(x, y, z, Urot=True):
    d = math.sqrt(x * x + y * y + z * z)
    if d != 0:
        i = math.acos(z / d)
    else:
        i = 0
    a = math.atan2(x, y)
    if Urot:
        return d, -a, Range180(i - PI / 2, PI)
    else:
        return d, a, i
    # https://en.wikipedia.org/wiki/Spherical_coordinate_system


def tangent_point(circle, circle_radius, point, angle_sign=1):

    circle, point = a2(circle), a2(point)

    # distance from the circle to the point
    circle_distance = dist2d(circle, point)

    # relative angle to tangent point
    post_angle = math.acos(Range(circle_radius / circle_distance, 1))

    # absolute angle to point
    point_angle = angle(point, circle)

    # absolute angle to tangent point
    tangent_angle = (point_angle - post_angle * sign(angle_sign))

    # tangent point coordinates
    tangent_x = math.cos(tangent_angle) * circle_radius + circle[0]
    tangent_y = math.sin(tangent_angle) * circle_radius + circle[1]

    return a2([tangent_x, tangent_y])


# minimum turning radius with no powerslide
def turning_radius(speed):
    return -6.901E-11 * speed**4 + 2.1815E-07 * speed**3 - 5.4437E-06 * speed**2 + 0.12496671 * speed + 157


def turning_speed(radius):
    return 10.219 * radius - 1.75404E-2 * radius**2 + 1.49406E-5 * radius**3 - 4.486542E-9 * radius**4 - 1156.05


# transforms local into global/world coordinates
def world(L, oL, oR, Urot=True):
    x, y, z = L
    tL = L
    if Urot:
        pitch = oR[0] * PI / U180
        yaw = Range180(oR[1] - U90, U180) * PI / U180
        roll = oR[2] * PI / U180
        R = np.array([pitch, yaw, roll])
    else:
        R = oR
    tL[0], tL[2] = rotate2D(x, z, R[2])
    tL[1], tL[2] = rotate2D(y, tL[2], R[0])
    tL[0], tL[1] = rotate2D(tL[0], tL[1], R[1])
    tL = tL + oL
    return tL


def world_2d(L, oL, ang):
    tL = rotate2D(*a2(L), -ang)
    return a2(tL) + a2(oL)


def writeFile(data, file):  # writes an object to a file
    pickle.dump(data, open(file, 'wb'))

def mag(vec):
    return math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)

def dist( v1, v2 ) :
    return mag( v1 - v2 )

class Triangle:
    def __init__( self, p1, p2, p3):
        self.p1 = flatten(p1)
        self.p2 = flatten(p2)
        self.p3 = flatten(p3)
        centroidx = (self.p1[0] + self.p2[0] + self.p3[0])/3
        centroidy = (self.p1[1] + self.p2[1] + self.p3[1])/3
        self.centroid = np.array([centroidx, centroidy, 0])
        #algorithm from https://keisan.casio.com/exec/system/1223428152

        a = dist(self.p1, self.p2)
        b = dist(self.p2, self.p3)
        c = dist(self.p1, self.p3)
        s = (a + b + c) / 2
        self.circle = math.sqrt((s*( s - a )*( s - b )*( s - c ))/s)

    def area(self, x1, y1, x2, y2, x3, y3 ) :

        return abs( (x1 * (y2 - y3) + x2 * (y3 - y1)
                     + x3 * (y1 - y2)) / 2.0 )

    def total_area( self ) :
        return abs( (self.p1[0] * (self.p2[1] - self.p3[1]) + self.p2[0] * (self.p3[1] - self.p1[1])
                     + self.p3[0] * (self.p1[1] - self.p2[1])) / 2.0 )

        # A function to check whether point P(x, y)

    # lies inside the triangle formed by
    # A(x1, y1), B(x2, y2) and C(x3, y3)
    def isInside( self, p ) :
        x = p[0]
        y = p[1]
        x1 = self.p1[0]
        y1 = self.p1[1]
        x2 = self.p2[0]
        y2 = self.p2[1]
        x3 = self.p3[0]
        y3 = self.p3[1]
        # Calculate area of triangle ABC
        A = self.area(  x1, y1, x2, y2, x3, y3 )

        # Calculate area of triangle PBC
        A1 = self.area( x, y, x2, y2, x3, y3 )

        # Calculate area of triangle PAC
        A2 = self.area( x1, y1, x, y, x3, y3 )

        # Calculate area of triangle PAB
        A3 = self.area( x1, y1, x2, y2, x, y )

        # Check if sum of A1, A2 and A3
        # is same as A
        if (A == A1 + A2 + A3) :
            return True
        else :
            return False
def flatten( v ) :
    return np.array( [v[0], v[1], 0] )
def confine_to_field( v ) :
    border = 190
    if abs( v[0] )<893 - border :
        return np.array( [max( -4096 + border, min( 4096 - border, v[0] ) ),
                     max( -5120 - 400 + border, min( 5120 + 400 - border, v[1] ) ),
                     max( -400 + border, min( 2044 + 400 - border, v[2] ) )] )
    else :
        return np.array( [max( -4096 + border, min( 4096 - border, v[0] ) ),
                     max( -5120 + border, min( 5120 - border, v[1] ) ), max( border, min( 2044 - border, v[2] ) )] )
def norm( v: vec3 ) :
    return v / mag( v )
class YeetControls( SimpleControllerState ) :
    def __init__( self ) :
        super( YeetControls, self ).__init__()

    @staticmethod
    def from_simple_controller( controls: SimpleControllerState ) :
        control = YeetControls()
        control.throttle = controls.throttle
        control.steer = controls.steer
        control.yaw = controls.yaw
        control.pitch = controls.pitch
        control.roll = controls.roll
        control.boost = controls.boost
        control.jump = controls.jump
        control.handbrake = controls.handbrake
        return control

    def __repr__( self ) :
        return "YeetControls(\nThrottle: " + '{0:.1g}'.format( self.throttle ) \
               + "\nSteer: " + '{0:.1g}'.format( self.steer ) \
               + "\nPitch: " + "{0:.1g}".format( self.pitch ) \
               + "\nYaw: " + "{0:.1g}".format( self.yaw ) \
               + "\nRoll: " + "{0:.1g}".format( self.roll ) \
               + "\nJump: " + "{0:.1g}".format( self.jump ) \
               + "\nHandbrake: " + "{0:.1g}".format( self.handbrake ) \
               + "\nBoost: " + "{0:.1g}".format( self.boost )

    def __len__( self ) :
        return 8

    def __getitem__( self, item ) :
        if item == 0 :
            return self.throttle
        elif item == 1 :
            return self.steer
        elif item == 2 :
            return self.yaw
        elif item == 3 :
            return self.pitch
        elif item == 4 :
            return self.roll
        elif item == 5 :
            return self.boost
        elif item == 6 :
            return self.jump
        elif item == 7 :
            return self.handbrake
        else :
            raise ValueError( "Not a control" )

    def to_array( self ) :
        return [
            self.throttle,
            self.steer,
            self.yaw,
            self.pitch,
            self.roll,
            self.boost,
            self.jump,
            self.handbrake
        ]

    def to_simple_controller_state( self ) :
        controls = SimpleControllerState()
        controls.throttle = self.throttle
        controls.steer = self.steer
        controls.yaw = self.yaw
        controls.pitch = self.pitch
        controls.roll = self.roll
        controls.boost = self.boost
        controls.jump = self.jump
        controls.handbrake = self.handbrake
        return controls

def cap(val, max, min):
    if(val>max):
        return max
    elif(val<min):
        return min
    return val

def turn_radius(v):
    if v == 0:
        return 0
    return 1.0 / curvature(v)
def curvature(v):
    if 0.0 <= v < 500.0:
        return 0.006900 - 5.84e-6 * v
    elif 500.0 <= v < 1000.0:
        return 0.005610 - 3.26e-6 * v
    elif 1000.0 <= v < 1500.0:
        return 0.004300 - 1.95e-6 * v
    elif 1500.0 <= v < 1750.0:
        return 0.003025 - 1.10e-6 * v
    elif 1750.0 <= v < 2500.0:
        return 0.001800 - 0.40e-6 * v
    else:
        return 0.0