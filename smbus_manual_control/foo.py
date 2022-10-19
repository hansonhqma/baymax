#!/usr/bin/env python
import smbus
import time
import typing
import enum
import math
import threading

I2C_ID = 0x15

ARM_SCALING_FACTOR = 700.0

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
                int( angle * ARM_SCALING_FACTOR + JOINT_OFFSET[i] ) )
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
        raw  = self.__mReadWord( ANGLE_READ_REG_ID )
        raw = ( raw >> 8 & 0xff ) | ( raw << 8 & 0xff00 )
        return ( raw - JOINT_OFFSET[ _aJointNum ] ) / ARM_SCALING_FACTOR

    def __mIOFunc( self ):
        joint_id = 0
        _aJointsRead = []
        count = 0
        initial = time.clock_gettime(0)
        while not self.__mShutdown:
            self.__mStartPosRead( joint_id )
            self.__mUpdateJoints( self.__mAngles )
            time.sleep( 0.002 )
            _aJointsRead.append(
                self.__mCompletePosRead( joint_id )
            )
            joint_id += 1
            if( joint_id == 6 ):
                joint_id = 0
                print( _aJointsRead )
                _aJointsRead = []
                print("Hz: " + str( count / (time.clock_gettime(0)-initial)))
            time.sleep( 0.002 )
            count += 1
        print("shutting down")

    def set_joints( self, _aJointAngles: typing.List[ float ] ):
        self.__mAngles = _aJointAngles

    def start( self ):
        self.__mBus = smbus.SMBus( 1 )
        self.__mAngles = [ 0.0 ] * 6
        self.__mShutdown = False
        self.__mIOThread = threading.Thread( target=self.__mIOFunc )
        self.__mIOThread.start()
    
    def stop( self ):
        print("stop")
        self.__mShutdown = True
        self.__mIOThread.join()

try:
    arm = Arm_Dev()
    arm.start()

    angle = 0.0
    incr = 0.005
    while True:
        arm.set_joints( [ angle ] * 6 )
        angle += incr
        if( math.fabs( angle ) > 0.2 ):
            incr = -incr

        time.sleep(0.01)

except KeyboardInterrupt:
    arm.stop()

'''
angle = 0.0
incr = 0.0005
initial = time.clock_gettime(0)
count = 1
while True:
    arm.set_joints( [ angle ] * 6 )
    angle += incr
    if( math.fabs( angle ) > 0.2 ):
        incr = -incr
    count += 1
    if( count % 1000 == 0 ):
        print( time.clock_gettime(0) - initial )
        initial = time.clock_gettime(0)
'''

