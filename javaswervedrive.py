from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState
from wpimath.trajectory import TrapezoidProfile
from ctre import * #WPI_TalonFX, FeedbackDevice, *
from ctre.sensors import * #WPI_Pigeon2, WPI_CANCoder

from wpilib import SmartDashboard

import math

class JavaSwerveDrive:
    kMaxSpeed = 3.0 # 3 meters per second
    kMaxAngularSpeed = math.pi # 1/2 rotation per second

    def __init__(self):
        self.m_frontLeftLocation = Translation2d(0.381, 0.381)
        self.m_frontRightLocation = Translation2d(0.381, -0.381)
        self.m_backLeftLocation = Translation2d(-0.381, 0.381)
        self.m_backRightLocation = Translation2d(-0.381, -0.381)

        self.m_frontLeft = JavaSwerveModule(7, 8, 18)
        self.m_frontRight = JavaSwerveModule(5, 6, 16)
        self.m_backLeft = JavaSwerveModule(3, 4, 14)
        self.m_backRight = JavaSwerveModule(1, 2, 12)

        self.m_gyro = WPI_Pigeon2(9, "rio")

        self.m_kinematics = SwerveDrive4Kinematics(
            self.m_frontLeftLocation,
            self.m_frontRightLocation,
            self.m_backLeftLocation,
            self.m_backRightLocation
        )

        self.m_odometry = SwerveDrive4Odometry(
            self.m_kinematics,
            self.m_gyro.getRotation2d(),
            [
                self.m_frontLeft.getPosition(),
                self.m_frontRight.getPosition(),
                self.m_backLeft.getPosition(),
                self.m_backRight.getPosition()
            ]
        )

        self.m_gyro.reset()

    def drive(self, xSpeed:float, ySpeed:float, rot:float, fieldRelative:bool=True):
        if fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rot,
                self.m_gyro.getRotation2d()
            )
        else:
            speeds = ChassisSpeeds(xSpeed, ySpeed, rot)    
        
        swerveModuleStates = self.m_kinematics.toSwerveModuleStates(speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
        self.m_frontLeft.setDesiredState(swerveModuleStates[0])
        self.m_frontRight.setDesiredState(swerveModuleStates[1])
        self.m_backLeft.setDesiredState(swerveModuleStates[2])
        self.m_backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self):
        pose = self.m_odometry.update(
            self.m_gyro.getRotation2d(),
            self.m_frontLeft.getPosition(),
            self.m_frontRight.getPosition(),
            self.m_backLeft.getPosition(),
            self.m_backRight.getPosition()
        )
        SmartDashboard.putNumberArray(
            "Field/JavaSwerveDrive",
            [pose.X(), pose.Y(), pose.rotation().degrees()]
        )

class JavaSwerveModule:
    kWheelRadius:float = 0.0508
    kDriveEncoderResolution:int = 2048
    kTurningEncoderResolution:int = 4096

    kModuleMaxAngularVelocity:float = JavaSwerveDrive.kMaxAngularSpeed
    kModuleMaxAngularAcceleration:float = 2 * math.pi; #// radians per second squared

    m_turningDistancePerPulse:float = 2 * math.pi / kTurningEncoderResolution
    m_driveDistancePerPulse:float = 2 * math.pi * kWheelRadius / kDriveEncoderResolution

    def __init__(self,
                 driveId:int,
                 turningId:int,
                 sensorId:int):
        self.m_drivePIDController:PIDController = PIDController(1, 0, 0)
        self.m_turningPIDController:ProfiledPIDController = ProfiledPIDController(
            1,
            0,
            0,
            TrapezoidProfile.Constraints(
                self.kModuleMaxAngularVelocity,
                self.kModuleMaxAngularAcceleration
            )
        )

        self.m_driveFeedforward:SimpleMotorFeedforwardMeters = SimpleMotorFeedforwardMeters(1, 3)
        self.m_turnFeedforward:SimpleMotorFeedforwardMeters = SimpleMotorFeedforwardMeters(1, 0.5)

        self.m_driveMotor = WPI_TalonFX(driveId, "canivore1")
        self.m_turningMotor = WPI_TalonFX(turningId, "canivore1")
        self.m_turningSensor = WPI_CANCoder(sensorId, "canivore1")

        self.m_driveMotor.configFactoryDefault()
        self.m_turningMotor.configFactoryDefault()
        self.m_turningSensor.configFactoryDefault()

        #cancodercfg = CANCoderConfiguration()
        #cancodercfg.sensorDirection = False
        #cancodercfg.sensorTimeBase.Per100Ms_Legacy = True
        #self.m_turningSensor.configAllSettings()

        self.m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        self.m_turningMotor.configRemoteFeedbackFilter(self.m_turningSensor,0)
        self.m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)

        self.m_driveMotor.setSensorPhase( False )
        self.m_turningMotor.setSensorPhase( False )
        self.m_turningSensor.configSensorDirection( False )

        self.m_driveEncoder = self.m_driveMotor.getSensorCollection()
        self.m_turningEncoder = self.m_turningMotor.getSensorCollection()
        
        self.m_turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def turnGetDistance(self) -> float:
        sensor = self.m_turningEncoder.getIntegratedSensorPosition()
        return (sensor * self.m_turningDistancePerPulse)

    def driveGetDistance(self) -> float:
        sensor = self.m_driveEncoder.getIntegratedSensorPosition()
        return (sensor * self.m_driveDistancePerPulse)

    def driveGetRate(self) -> float:
        sensor = self.m_driveEncoder.getIntegratedSensorVelocity()
        return (sensor * self.m_driveDistancePerPulse)
      
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.driveGetRate(),
            Rotation2d(self.turnGetDistance())
        )

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.driveGetDistance(),
            Rotation2d(self.turnGetDistance())
        )

    def setDesiredState(self,desiredState:SwerveModuleState) -> None:
        state:SwerveModuleState = SwerveModuleState.optimize(
            desiredState,
            Rotation2d(self.turnGetDistance())
        )

        driveOutput:float = self.m_drivePIDController.calculate(
            self.driveGetRate(),
            state.speed
        )
        driveFeedforward:float = self.m_driveFeedforward.calculate(
            state.speed
        )

        turnOutput:float = self.m_turningPIDController.calculate(
            self.turnGetDistance(),
            state.angle.radians()
        )
        m_turningPID_setPoint:TrapezoidProfile.State = self.m_turningPIDController.getSetpoint()
        turnFeedforward:float = self.m_turnFeedforward.calculate(
            m_turningPID_setPoint.velocity
        )

        self.m_driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.m_turningMotor.setVoltage(turnOutput + turnFeedforward)
