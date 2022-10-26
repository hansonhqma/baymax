#!/usr/bin/python3
import smbus
import random
import time
import typing
import enum
import math
import threading

I2C_ID = 0x15

JOINT_SCALING_FACTOR = (
    [
        700.0,
        -700.0,
        -700.0,
        -700.0,
        700.0,
        700.0
    ]
)

JOINT_OFFSET = (
    [
        2000.0,
        2000.0,
        2000.0,
        2000.0,
        1466.0,
        2000.0,
    ]
)

TIME_REG_ID = 0x1e
ANGLE_WRITE_REG_ID = 0x1d
ANGLE_READ_REG_ID = 0x37
TORQUE_REG_ID = 0x1A

class Arm_Dev:
    def __mWrite( self, _mReg: int, _mData: typing.List[ int ] ):
        self.__mBus.write_i2c_block_data( I2C_ID, _mReg, _mData )

    def __mWriteByte( self, _mReg: int, _mData: int ):
        self.__mBus.write_byte_data( I2C_ID, _mReg, _mData )

    def __mReadWord( self, _mReg: int ) -> int:
        return self.__mBus.read_word_data( I2C_ID, _mReg )

    # Set joint angles in units of radians from center
    def __mUpdateJoints( self, _aJointAngles: typing.List[ float ] ):
        joint_angles_int = []
        i = 0
        for angle in _aJointAngles:
            joint_angles_int.append(
                int( angle * JOINT_SCALING_FACTOR[i] + JOINT_OFFSET[i] ) )
            i += 1
        
        data_buf = []
        for angle in joint_angles_int:
            data_buf.append( ( angle >> 8 ) & 0xff )
            data_buf.append( angle & 0xff )

        self.__mWrite( TIME_REG_ID, [ 0x0, 0x0 ] )
        self.__mWrite( ANGLE_WRITE_REG_ID, data_buf )
    
    def __mStartPosRead( self, _aJointNum: int ):
        self.__mWriteByte( ANGLE_READ_REG_ID, _aJointNum + 1 )
    
    def __mCompletePosRead( self, _aJointNum: int ) -> float:
        raw = self.__mReadWord( ANGLE_READ_REG_ID )
        raw = ( raw >> 8 & 0xff ) | ( raw << 8 & 0xff00 )
        return ( raw - JOINT_OFFSET[ _aJointNum ] ) / JOINT_SCALING_FACTOR[ _aJointNum ]

    def __mUpdateTorque( self, _aTorque: bool ):
        if _aTorque:
            self.__mWriteByte( TORQUE_REG_ID, 0x01 )
        else:
            self.__mWriteByte( TORQUE_REG_ID, 0x00 )

    def __mIOFunc( self ):
        joint_id = 0
        _JointsRead = [ 0 ] * 6
        _JointsSkipped = [ 0 ] * 6
        while not self.__mShutdown:
            if self.__mSetTorque != self.__mCurrTorque:
                self.__mUpdateTorque( self.__mSetTorque )
                self.__mCurrTorque = self.__mSetTorque
            self.__mStartPosRead( joint_id )
            self.__mUpdateJoints( self.__mAngles )
            time.sleep( 0.002 )
            temp_j = self.__mCompletePosRead( joint_id )
            if( math.fabs( temp_j - _JointsRead[joint_id] ) < 1 or
                _JointsSkipped[joint_id] > 0 ):
                _JointsSkipped[joint_id] = 0
                _JointsRead[joint_id] = temp_j
            else:
                print("Single value switch (joint " + str( joint_id ) + "): " + str(math.fabs( temp_j - _JointsRead[joint_id] ) ) )
                _JointsSkipped[joint_id] += 1
            joint_id += 1
            if( joint_id == 6 ):
                joint_id = 0
                self.__mAngleReadHandler( _JointsRead )
            time.sleep( 0.002 )

    def set_torque( self, _aTorque: bool ):
        self.__mSetTorque = _aTorque

    def set_joints( self, _aJointAngles: typing.List[ float ] ):
        self.__mAngles = _aJointAngles

    def start( self, _aAngleReadHandler: typing.Callable ):
        self.__mSetTorque = False
        self.__mCurrTorque = None
        self.__mBus = smbus.SMBus( 1 )
        self.__mAngleReadHandler = _aAngleReadHandler
        self.__mAngles = [ 0.0 ] * 6
        self.__mShutdown = False
        self.__mIOThread = threading.Thread( target=self.__mIOFunc )
        self.__mIOThread.start()
    
    def stop( self ):
        self.__mShutdown = True
        self.__mIOThread.join()


if __name__ == "__main__":
    def handle_angles( angles ):
        print( angles )

    try:
        arm = Arm_Dev()
        arm.start( handle_angles )

        angle = 0.0
        incr = 0.0001
        while True:
            arm.set_joints( [ angle ] * 6 )
            angle += incr
            if( math.fabs( angle ) > 0.2 ):
                incr = -incr

            time.sleep(0.001)

    except KeyboardInterrupt:
        arm.stop()
