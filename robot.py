from wpilib import *
from wpimath import applyDeadband

from basicswervedrive import BasicSwerveDrive
from javaswervedrive import JavaSwerveDrive

class Robot(TimedRobot):
    __deadband = 0.005
    __drivetrain = 1

    def robotInit(self):
        self.op1 = XboxController(0)
        self.swerve1 = BasicSwerveDrive()
        self.swerve2 = JavaSwerveDrive()
        #self.swerve3 = AdvSwerveDrive()

        SmartDashboard.putNumber( "DriveTrainVersion", self.__drivetrain )

    def robotPeriodic(self):
        self.__drivetrain = SmartDashboard.getNumber( "DriveTrainVersion", 1 )
        self.swerve2.updateOdometry()

    def teleopPeriodic(self):
        pass

    def testPeriodic(self):
        x1 = applyDeadband( self.op1.getLeftX(), self.__deadband, 1 )
        y1 = applyDeadband( self.op1.getLeftY(), self.__deadband, 1 )
        x2 = applyDeadband( self.op1.getRightX(), self.__deadband, 1 )

        if self.__drivetrain == 1: self.swerve1.drive(x1,y1,x2)
        elif self.__drivetrain == 2: self.swerve2.drive(x1,y1,x2,True)
        #elif self.__drivetrain == 3: self.swerve3.drive(x1,y1,x2)

### Start Robot When Loading This File
if __name__ == '__main__':
    run(Robot)