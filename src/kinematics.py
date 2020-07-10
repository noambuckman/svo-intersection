import numpy as np

class Pose:
    def __init__(self,x,y,phi=None,z=None):
        self.x = x
        self.y = y
        if phi==None:
            self.phi = 0
        else:
            self.phi = phi

    def dist(self,other):
        d1 = (self.x-other.x)**2+(self.y-other.y)**2
        return d1**(0.5)

    def __str__(self):
        return "X=" + '%.03f'%(self.x) + ", Y=" + '%.03f'%(self.y)

    def __eq__(self,other):
        return type(self) == type(other) and self.dist(other)<0.0000001


# Some functions to make geometries
def posesCircle(nr,radius):
    #Return list of poses evenly spaced with radius commRadius
    R = radius
    return [Pose(R*np.cos(2*i*np.pi/nr),R*np.sin(2*i*np.pi/nr)) for i in range(nr)]


def posesRing(nr,commRadius):
    #Return list of poses evenly spaced with radius commRadius
    R = (0.95*commRadius)/np.sqrt(2-2*np.cos(2*np.pi/nr))
    return posesCircle(nr,R)
    # return [Pose(R*np.cos(2*i*np.pi/nr),R*np.sin(2*i*np.pi/nr)) for i in range(nr)]

def posesLine(nr,commRadius):
    R = 0.95*commRadius
    return [Pose(R*(i-(nr-1)/2.0),0) for i in range(nr)]


def posesRingFC(nr,commRadius):
    R = 0.45*commRadius
    return posesCircle(nr,R)

def posesStar(nr,commRadius):
    centerPose = Pose(0,0)
    perimPoses = posesCircle(nr-1,0.95*commRadius)
    return [centerPose] + perimPoses
