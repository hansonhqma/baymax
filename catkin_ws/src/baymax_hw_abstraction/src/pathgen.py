import numpy as np
import matplotlib.pyplot as plt
import typing

def squared_velocity_path(lamda, lamda_max, pos_delta):
    c = pos_delta*3/(2*(lamda_max/2)**3)
    if lamda <= lamda_max/2:
        return c*lamda**2
    else:
        return c*(lamda-lamda_max)**2

def plt_sphere(ax, c, r):
    # draw sphere
    u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
    x = r*np.cos(u)*np.sin(v)
    y = r*np.sin(u)*np.sin(v)
    z = r*np.cos(v)

    ax.plot_surface(x+c[0], y+c[1], z+c[2], color=np.random.choice(['g','b']), alpha=0.5*np.random.random()+0.5)

class obstacle:
    __Origin: np.ndarray
    __Radius: float
    __RepulsiveFactor: float
    __BufferSize: float
    def __init__(
        self,
        _aOrigin: np.ndarray,
        _aRadius: float,
        _aRepulsiveFactor: float = 0.1,
        _aBufferSize: float = 0.05
    ):
        self.__Origin = _aOrigin
        self.__Radius = _aRadius
        self.__RepulsiveFactor = _aRepulsiveFactor
        self.__BufferSize = _aBufferSize
    
    def get_repulsive_force( self, _aLoc ):
        dist_to_obstacle = np.linalg.norm( self.__Origin - _aLoc ) - self.__Radius
        if dist_to_obstacle < 0:
            raise Exception()
        
        if dist_to_obstacle > self.__BufferSize:
            return np.zeros( (3,) )

        return (
            self.__RepulsiveFactor *
            ( 1 / dist_to_obstacle - 1 / self.__BufferSize ) *
            ( _aLoc - self.__Origin ) / dist_to_obstacle
        )
    def plot( self, ax ):
        plt_sphere( ax, self.__Origin, self.__Radius )

def generate_path (
    _aOrigin: np.ndarray,
    _aDestination: np.ndarray,
    _aObstacles: typing.List[ obstacle ],
    _aNumSteps: int,
    _aAttractiveForce: float = 0.1,
    _aAlpha: float = 0.1,
    _aEpsilonError: float = 1e-1
):
    GradientPath = [ _aOrigin ]
    while np.linalg.norm( GradientPath[-1] - _aDestination ) >= _aEpsilonError:
        Force = -_aAttractiveForce * ( GradientPath[-1] - _aDestination )

        for obstacle in _aObstacles:
            r = obstacle.get_repulsive_force( GradientPath[-1] )
            Force += r

        GradientPath.append( GradientPath[-1] + _aAlpha * Force )

    GradientPath = np.array( GradientPath )

    print( GradientPath )

#    for i in range( 3 ):
#        plt.plot( GradientPath[:,i] )
#    plt.show()
    ax = plt.axes(projection='3d')
    for o in _aObstacles:
        o.plot( ax )
    ax.scatter3D(
        GradientPath[:,0],
        GradientPath[:,1],
        GradientPath[:,2]
    )
    ax.set_aspect('equal')
    plt.show()

    PathDistance = 0
    for i in range( 1, GradientPath.shape[0] ):
        PathDistance += np.linalg.norm( GradientPath[i-1] - GradientPath[i] )

    print(PathDistance)

    RealPath = [ _aOrigin ]

    print(np.linalg.norm(GradientPath[0]-GradientPath[1]))

    i = 1
    for j in range(0, _aNumSteps):
        dist_left = squared_velocity_path( j, _aNumSteps, PathDistance )
        curr_loc = RealPath[-1]

        while np.linalg.norm( curr_loc - GradientPath[i] ) < dist_left:
            dist_left -= np.linalg.norm( curr_loc - GradientPath[i] )
            curr_loc = GradientPath[i]
            i += 1
            if i == GradientPath.shape[0]:
                break

        if i == GradientPath.shape[0]:
            RealPath.append( curr_loc )
            break
        else:
            RealPath.append( curr_loc + dist_left * ( GradientPath[i] - curr_loc ) / np.linalg.norm( GradientPath[i] - curr_loc ) )

    RealPath = np.array( RealPath )

    print(RealPath)

    ax = plt.axes(projection='3d')
    for o in _aObstacles:
        o.plot( ax )
    ax.scatter3D(
        RealPath[:,0],
        RealPath[:,1],
        RealPath[:,2]
    )
    ax.set_aspect('equal')
    plt.show()

#    for i in range( 3 ):
#        plt.plot( RealPath[:,i] )
#    plt.show()

if __name__=="__main__":
    o = ( [
        obstacle( 
        np.array( [0.5,0.5,0.5] ),
        0.9,
        0.1,
        0.5 ),
        obstacle( 
        np.array( [0.3,-0.5,-0.5] ),
        0.9,
        0.1,
        0.5 ),
    ] )
    generate_path(
        np.array( [-1,1,2] ),
        np.array( [1,-1,-2] ),
        o,
        100,
    )
