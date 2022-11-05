#!/usr/bin/python3
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from std_msgs.msg import Float32MultiArray
import time
import math

alpha = 0.4
iterations = 70

class robot:
    def __init__( self, _aAxes: np.ndarray, _aTranslations: np.ndarray ):
        # Make sure each argument is the right size matrix
        assert( _aAxes.shape[0] == _aTranslations.shape[0] )
        assert( _aAxes.shape[1] == 3 )
        assert( _aTranslations.shape[1] == 3 )
        # Save robot specific attributes
        self.__mNumJoints = _aAxes.shape[0]
        self.__mAxes = _aAxes
        self.__mTranslations = _aTranslations

    def fk( self, _aAngles: np.ndarray ):
        # Make sure the size of the angles vector is correct
        assert( _aAngles.shape[0] == self.__mNumJoints )

        # Create the rotation matrices between each joint from
        # the axis angle representation
        R_ij = Rotation.from_rotvec(
            np.multiply(
                np.asarray( [ _aAngles ] * 3 ).T,
                self.__mAxes
            )
        )

        # Multiply each rotation to find the rotation matrix from
        # the base frame to the end effector frame
        R_0i = R_ij
        for i in range( 1, self.__mNumJoints ):
            R_0i[i] = R_0i[ i-1 ] * R_ij[ i ]

        # Determine the translation vectors to each joint in this configuraiton
        P_0i = np.ndarray( ( self.__mNumJoints, 3 ) )
        P_0i[0] = self.__mTranslations[0]
        for i in range( 1, self.__mNumJoints ):
            P_0i[ i ] = np.add(
                P_0i[ i-1 ],
                R_0i[ i-1 ].apply( self.__mTranslations[i] )
            )

        # Return the rotation matrices and translation vectors
        return R_0i, P_0i

    # Returns the Jacobian for a given configuration    
    def Jq( self, _aAngles: np.ndarray ):
        # Perform the forward kinematics
        R_0i, P_0i = self.fk( _aAngles )
        # Find the axes of each joint in this configuration
        hi_0 = np.ndarray( ( self.__mNumJoints, 3 ) )
        hi_0[0] = self.__mAxes[0]
        for i in range( 1, self.__mNumJoints ):
            hi_0[i] = R_0i[i-1].apply( self.__mAxes[i] )

        # Find the distance from each joint to the end effector        
        P_iT_0 = np.ndarray( ( self.__mNumJoints, 3 ) )
        for i in range( 0, self.__mNumJoints ):
            P_iT_0[i] = P_0i[-1] - P_0i[i]
        
        # Generate the Jacobian
        Jq = np.ndarray( ( 6, self.__mNumJoints ) )
        for i in range( 0, self.__mNumJoints ):
            Jq[0:3,i] = hi_0[i]
            Jq[3:6,i] = np.cross( hi_0[i], P_iT_0[i] )

        return Jq, R_0i, P_0i

    def ik( self, _aTargetRot, _aTargetTrans, _aStartingAngles ):
        angles = _aStartingAngles
        for i in range( 0, iterations ):
            # Perform forward kinematics and get the jacobian for
            # this configuration
            Jq, R_0i, P_0i = self.Jq( angles )
            # Calculate the error in rotation matrices
            R_err = R_0i[-1] * _aTargetRot.inv()
            # Calculate the error in translation
            P_err = P_0i[-1] - _aTargetTrans
            # Put both of these values into a single error vector
            dx = np.ndarray( (6,) )
            dx[0:3] = R_err.as_rotvec()
            dx[3:6] = P_err

            # Perform the "pseudo inverse" of the jacobian with the
            # damped least squares method
            inv = np.matmul(
                Jq.T,
                np.linalg.inv(
                    np.add(
                        np.matmul( Jq, Jq.T ),
                        0.001 * np.identity( Jq.shape[0] )
                    )
                )
            )

            # Update angles according to the update matrix
            # and the error matrix
            angles = np.add(
                angles,
                - alpha * np.matmul( 
                    #np.linalg.pinv( Jq ),
                    inv,
                    dx )
            )
        
        print( "Error: " + str( np.linalg.norm( dx ) ) )

        return angles

# Initialize the robot with the corresponding joints etc
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

# Ros initialization
NODE_NAME = "foo"
SET_JOINT_MSG = "/base/set_joints"

rospy.init_node( NODE_NAME )
pub = rospy.Publisher( SET_JOINT_MSG, Float32MultiArray, queue_size=10 )

# Perform inverse kinematics
rate = rospy.Rate(10)

count = 0
angles = np.array( [0,0,0,0,0] )
while True:
    angles = r.ik(
        Rotation.from_rotvec( [ 0.3 * math.cos( count / 10 ), -0.3 * math.sin( count / 10 ), 0 ] ),
        #Rotation.from_rotvec( [ 0, 0, -count / 10 ] ),
        #np.array( [ 0.15 * math.cos( count / 10 ), 0.15 * math.sin( count / 10 ), 0.25 + 0.05 * math.sin( count / 7 ) ] ),
        np.array( [ 0, 0, 0.35 + 0.03 * math.sin( count / 7 ) ] ),
        angles
    )
    msg = Float32MultiArray()
    msg.data = tuple( angles ) + (0,)
    pub.publish( msg )
    count += 1
    rate.sleep()

