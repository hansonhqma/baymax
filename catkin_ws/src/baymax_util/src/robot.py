#!/usr/bin/python3
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from std_msgs.msg import Float32MultiArray
import time
import math

class DEFAULT:
    ITERATIONS = 2000
    # Learning rate
    ALPHA = 1
    # Used for damped least squares method
    EPSILON_REGULARIZATION = 0.01
    # When the norm of the error matrix is less
    # than this scalar value, inverse kinematics
    # are said to have converged
    EPSILON_ERROR = 5e-3
    # Beta is a elementwise scaling vector of the
    # error term. Recall the first three terms are
    # the 3d translational error, which is chosen
    # as reference, and thus corresponding beta
    # terms are set to one
    #
    # The second two represent pitch and roll
    # rotations. An error of 10 cm ( 0.1m ) was
    # deemed equivalent to an error of pi / 4 rad.
    # Thus scaling factors for these terms are
    # calculated below
    #
    # The last term corresponds to yaw error,
    # which is ignored since it is impossible to
    # fix based on the construction of the dofbot
    """
    BETA = np.array( 
        [ 0.1 / ( math.pi / 4 ),
          0.1 / ( math.pi / 4 ),
          0,
          1, 1, 1 ] )
    """
    BETA = np.array(
        [0,
        0,
        0,
        1,
        1,
        1]
    )
    JOINT_LIMIT = math.pi / 2

class robot:
    def __init__( self,
                  _aAxes: np.ndarray,
                  _aTranslations: np.ndarray,
                  _aUpperLimit: np.ndarray = None,
                  _aLowerLimit: np.ndarray = None
                ):
        # Make sure each argument is the right size matrix
        assert( _aAxes.shape[0] == _aTranslations.shape[0] )
        assert( _aAxes.shape[1] == 3 )
        assert( _aTranslations.shape[1] == 3 )
        # Save robot specific attributes
        self.__mNumJoints = _aAxes.shape[0]
        self.__mAxes = _aAxes
        self.__mTranslations = _aTranslations

        if _aUpperLimit == None:
            self.__mUpperLimit = np.linspace(
                DEFAULT.JOINT_LIMIT,
                DEFAULT.JOINT_LIMIT,
                self.__mNumJoints
            )
        else:
            assert( _aUpperLimit.shape == (self.__mNumJoints,) )
            self.__mUpperLimit = _aUpperLimit

        if _aLowerLimit == None:
            self.__mLowerLimit = np.linspace(
                -DEFAULT.JOINT_LIMIT,
                -DEFAULT.JOINT_LIMIT,
                self.__mNumJoints
            )
        else:
            assert( _aLowerLimit.shape == (self.__mNumJoints,) )
            self.__mLowerLimit = _aLowerLimit
    
    def __copy__( self ):
        return robot( self.__mAxes, self.__mTranslations )

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

    # 
    def ik( self,
            _aTargetRot: Rotation,
            _aTargetTrans: np.ndarray,
            _aStartingAngles: np.ndarray,
            _aIterations=DEFAULT.ITERATIONS,
            _aAlpha=DEFAULT.ALPHA,
            _aEpsilonRegularization=DEFAULT.EPSILON_REGULARIZATION,
            _aEpsilonError=DEFAULT.EPSILON_ERROR,
            _aBeta=DEFAULT.BETA ):
        angles = _aStartingAngles
        for i in range( 0, _aIterations ):
            # Perform forward kinematics and get the jacobian for
            # this configuration
            Jq, R_0i, P_0i = self.Jq( angles )
            # Calculate the error in rotation matrices
            R_err = R_0i[-1] * _aTargetRot.inv()
            # Calculate the error in translation
            P_err = P_0i[-1] - _aTargetTrans
            # Put both of these values into a single error vector
            err = np.ndarray( (6,) )
            err[0:3] = R_err.as_euler( "xyz" )
            err[3:6] = P_err
            err = np.multiply( err, _aBeta )

            if( np.linalg.norm( err ) < _aEpsilonError ):
                return angles

            # Perform the "pseudo inverse" of the jacobian with the
            # damped least squares method
            inv = np.matmul(
                Jq.T,
                np.linalg.inv(
                    np.add(
                        np.matmul( Jq, Jq.T ),
                        _aEpsilonRegularization * np.identity( Jq.shape[0] )
                    )
                )
            )

            # Update angles according to the update matrix
            # and the error matrix
            angles = np.add(
                angles,
                - _aAlpha * np.matmul( 
                    inv,
                    err )
            )

            for i in range( 0, self.__mNumJoints ):
                if angles[i] > self.__mUpperLimit[i] or angles[i] < self.__mLowerLimit[i]:
                    angles[i] = self.__mUpperLimit[i] + self.__mLowerLimit[i]

        raise Exception( "Could not converge!" )

DOFBOT = robot(
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

if __name__ == "__main__":
    # Ros initialization
    NODE_NAME = "foo"
    SET_JOINT_MSG = "/base/set_joints"

    rospy.init_node( NODE_NAME )
    pub = rospy.Publisher( SET_JOINT_MSG, Float32MultiArray, queue_size=10 )

    # Perform inverse kinematics
    rate = rospy.Rate(10)

    """
    angles = DOFBOT.ik(
        Rotation.from_euler( "xyz", [0.1,math.pi/2,0] ),
        np.array( [ 0.2, 0, 0.2 ] ),
        np.array( [0.1,0.1,0.1,0.1,0.1] )
    )
    print(angles)
    msg = Float32MultiArray()
    msg.data = tuple( angles ) + (0,)
    pub.publish( msg )

    exit(0)
    """

    angles = np.array( [0.1,0.1,0.1,0.1,0.1] )
    count = 1

    while not rospy.is_shutdown():
        """
        angles = DOFBOT.ik(
            Rotation.from_rotvec( [ 0.3 * math.cos( count / 10 ), -0.3 * math.sin( count / 10 ), 0 ] ),
            np.array( [ 0, 0, 0.35 + 0.03 * math.sin( count / 7 ) ] ),
            angles
        )
        """
        y = 0.1 * math.sin( count / 25 )
        if( math.fabs( y ) < 0.05 ):
            z = math.sqrt( 0.05 ** 2 - y ** 2 )
        else:
            z = 0
        angles = DOFBOT.ik(
            Rotation.from_rotvec( [ 0, 0, 0 ] ),
            np.array( [ 0.23, 0.1 * math.sin( count / 25 ), z ] ),
            angles
        )

        msg = Float32MultiArray()
        msg.data = tuple( angles ) + (0,)
        pub.publish( msg )
        count += 1
        rate.sleep()
