from ctre import WPI_TalonFX
from ctre.sensors import WPI_CANCoder, WPI_Pigeon2
from wpimath.controller import PIDController
import math
from wpilib import SmartDashboard

class BasicSwerveDrive:
    __length = 0.5
    __width = 0.5
    
    def __init__(self):
        self.__gyro = WPI_Pigeon2(9, "rio")
        self.__moduleFL = BasicSwerveModule("FL", 7, 8, 18)
        self.__moduleFR = BasicSwerveModule("FR", 5, 6, 16)
        self.__moduleBL = BasicSwerveModule("BL", 3, 4, 14)
        self.__moduleBR = BasicSwerveModule("BR", 1, 2, 12)

    def drive(self, x1, y1, x2):
        r = math.sqrt( (self.__length * self.__width) + (self.__width * self.__width))
        y1 *= -1

        a = x1 - x2 * (self.__length / r)
        b = x1 + x2 * (self.__length / r)
        c = y1 - x2 * (self.__width / r)
        d = y1 + x2 * (self.__width / r)

        flSpeed = math.sqrt( (b*b) + (c*c) )
        frSpeed = math.sqrt( (b*b) + (d*d) )
        blSpeed = math.sqrt( (a*a) + (c*c) )
        brSpeed = math.sqrt( (a*a) + (d*d) )

        flAngle = math.atan2(b, c) / math.pi
        frAngle = math.atan2(b, d) / math.pi
        blAngle = math.atan2(a, c) / math.pi
        brAngle = math.atan2(a, d) / math.pi

        self.__moduleFL.drive(flSpeed, flAngle)
        self.__moduleFR.drive(frSpeed, frAngle)
        self.__moduleBL.drive(blSpeed, blAngle)
        self.__moduleBR.drive(brSpeed, brAngle)

class BasicSwerveModule:
    __maxVolts = 4.95
    
    def __init__(self, name, driveId, angleId, sensorId):
        # Set Private Variables
        self.__name = name
        self.__driveMotor = WPI_TalonFX(driveId, "canivore1")
        self.__angleMotor = WPI_TalonFX(angleId, "canivore1")
        self.__angleSensor = WPI_CANCoder(sensorId, "canivore1")
        self.__pidController = PIDController(1, 0, 0)

        #self.__pidController.setIntegratorRange(-1, 1) #.setOutputRange( -1, 1 )
        self.__pidController.enableContinuousInput(-1,1)
        
        SmartDashboard.putData(f"{self.__name}: PID", self.__pidController)
    
    def drive(self, speed, angle):
        self.__driveMotor.set(speed)

        setpoint = angle * (self.__maxVolts * 0.5) + (self.__maxVolts * 0.5)
        if setpoint < 0:
            setpoint = self.__maxVolts + setpoint
        if setpoint > self.__maxVolts:
            setpoint = setpoint - self.__maxVolts

        self.__pidController.setSetpoint(setpoint)

        newAngle = self.__pidController.calculate( angle )
        self.__angleMotor.set(newAngle)

        SmartDashboard.putData(f"{self.__name}: PID", self.__pidController)

        