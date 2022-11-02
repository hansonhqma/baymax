#!/usr/bin/python3
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from std_msgs.msg import Float32MultiArray
import time
import math

alpha = 0.5
iterations = 10000

class robot:
    def __init__( self, _aAxes: np.ndarray, _aTranslations: np.ndarray ):
        assert( _aAxes.shape[0] == _aTranslations.shape[0] )
        assert( _aAxes.shape[1] == 3 )
        assert( _aTranslations.shape[1] == 3 )
        self.__mNumJoints = _aAxes.shape[0]
        self.__mAxes = _aAxes
        self.__mTranslations = _aTranslations

    def fk( self, _aAngles: np.ndarray ):
        assert( _aAngles.shape[0] == self.__mNumJoints )

        R_ij = Rotation.from_rotvec(
            np.multiply(
                np.asarray( [ _aAngles ] * 3 ).T,
                self.__mAxes
            )
        )

        R_0i = R_ij
        for i in range( 1, self.__mNumJoints ):
            R_0i[i] = R_0i[ i-1 ] * R_ij[ i ]

        P_0i = np.ndarray( ( self.__mNumJoints, 3 ) )
        P_0i[0] = self.__mTranslations[0]
        for i in range( 1, self.__mNumJoints ):
            P_0i[ i ] = np.add(
                P_0i[ i-1 ],
                R_0i[ i-1 ].apply( self.__mTranslations[i] )
            )

        return R_0i, P_0i
    
    def Jq( self, _aAngles: np.ndarray ):
        R_0i, P_0i = self.fk( _aAngles )
        hi_0 = np.ndarray( ( self.__mNumJoints, 3 ) )
        hi_0[0] = self.__mAxes[0]
        for i in range( 1, self.__mNumJoints ):
            hi_0[i] = R_0i[i-1].apply( self.__mAxes[i] )
        
        P_iT_0 = np.ndarray( ( self.__mNumJoints, 3 ) )
        for i in range( 0, self.__mNumJoints ):
            P_iT_0[i] = P_0i[-1] - P_0i[i]
        
        Jq = np.ndarray( ( self.__mNumJoints, 6 ) )
        for i in range( 0, self.__mNumJoints ):
            Jq[i,0:3] = hi_0[i]
            Jq[i,3:6] = np.cross( hi_0[i], P_iT_0[i] )

        return Jq, R_0i, P_0i

    def ik_pseudo_inv( self, _aTargetRot, _aTargetTrans, _aStartingAngles ):
        angles = _aStartingAngles
        print( angles )
        for i in range( 0, iterations ):
            msg = Float32MultiArray()
            msg.data = tuple( angles ) + (0,)
            print("q")
            print( msg.data )
            pub.publish( msg )

            Jq, R_0i, P_0i = self.Jq( angles )
            R_err = R_0i[-1] * _aTargetRot.inv()
            print( P_0i )

            """
            print("Rotations:")
            print( R_0i.as_rotvec() )
            print( R_0i[-1:].as_rotvec() )
            print( _aTargetRot.as_rotvec() )
            print(R_err.as_rotvec())
            """

            P_err = P_0i[-1] - _aTargetTrans
            dx = np.ndarray( (6,) )
            dx[0:3] = R_err.as_rotvec()
            dx[3:6] = P_err

            print("Translations")
            print(_aTargetTrans)
            print(P_0i[-1])
            print(P_err)
            print("dx")
            print( dx )
            print("matmul")
            print( np.matmul( Jq, dx ) )
            print("Jq")
            print( Jq )

            angles = np.add(
                angles,
                - alpha * np.matmul( Jq, dx )
            )
            print(0.1)

        pass

r = robot(
    np.array(
      [ [ 0,  0, 1 ],
        [ 0, -1, 0 ],
        [ 0, -1, 0 ],
        [ 0, -1, 0 ],
        [ 0,  0, 1 ] ] ),
    np.array(
      [ [ 0,  0, 0.1075 ],
        [ 0, 0, 0       ],
        [ 0, 0, 0.08285 ],
        [ 0, 0, 0.08285 ],
        [ 0, 0, 0.12842 ] ] )
)

NODE_NAME = "foo"
SET_JOINT_MSG = "/base/set_joints"

rospy.init_node( NODE_NAME )
pub = rospy.Publisher( SET_JOINT_MSG, Float32MultiArray, queue_size=10 )

print( r.Jq( np.array( [ math.pi, math.pi/2, -math.pi/2, math.pi/2, -math.pi ] ) ) )

rot, trans = r.fk( np.array( [0,0,0,0,0] ) )

r.ik_pseudo_inv(
    Rotation.from_rotvec( np.array( [ 0, 1.57, 0 ] ) ),
    np.array( [ 0.3, 0, 0.01 ] ),
    np.array( [ 0, 0.1, -0.2, 0.05, 0 ] )
)
