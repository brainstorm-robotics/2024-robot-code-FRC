// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode; // <<< INVESTIGATE <<<
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

//import frc.robot.subsystems.sensors.FRCGyro;
//import frc.robot.subsystems.sensors.InfraredDistanceSensor;
//import frc.robot.subsystems.sensors.LimitSwitch;
import frc.robot.subsystems.sensors.NavX2Gyro;
//import frc.robot.subsystems.sensors.UltrasonicDistanceSensor;

import frc.utils.PIDSettings;
import frc.utils.PIDSettings2;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //int kDriverControllerPort = 0;

  /**
   * a class with static constants and related definitions for the shooter
   */
  public static final class Climber {

    //public static final int kClimberLeftCanId  = 61;
    //public static final int kClimberRightCanId = 62;

    public static final double  kVelocity        = 5737.0; // ticks per 100ms
    public static final double  kAcceleration    = 2295.0; // ticks per 100ms per s
    
    public static PIDSettings2 climberLeftPIDSettings = new PIDSettings2(
      0.1, 1.0e-4, 1.0, 0.0, -kVelocity, kVelocity, "Climber (Left)");

      public static PIDSettings2 climberRightPIDSettings = new PIDSettings2(
        0.1, 1.0e-4, 1.0, 0.0, -kVelocity, kVelocity, "Climber (Right)");

    public static final double  kDeadband        = 0.001;
    public static final boolean kInvertedLeft    = false;
    public static final boolean kInvertedRight   = false;
    public static final double  kNominalForward  = 0.0;
    public static final double  kNominalBackward = 0.0;
  
    public static final double  kWinchRevolutions    = 9.34; // to extend climber
    public static final double  kMechanicalAdvantage = 15.0;

    public static final double  kAngleTolerance  = 10; // in degrees
    public static final double  kClimbTolerance  = 300; // in ticks

  } // end class Climber



  /**
   * a class with static constants and related definitions for the shooter
   */
  public static final class Intake {

    public static final int kIntakeCanId        = 51;
    public static final int kIntakeRotatorCanId = 52;

    public static PIDSettings intakePIDSettings = new PIDSettings(
      0.1, 1.0e-4, 1.0, 0.0, 0.0, -1, 1.0, "Intake");

    public static PIDSettings intakeRotatorPIDSettings = new PIDSettings(
      0.1, 1.0e-4, 1.0, 0.0, 0.0, -1.0, 1.0, "Intake Rotor");

    public static final double MAX_ARM_ROTATION = 0.485;//0.5
    public static final double MIN_ARM_ROTATION = 0.01;//0.1

    public static final double kIntakeSpeed      = 0.4; // negative for intake a note
    public static final double kFeedShooterSpeed =  -0.4; // positive for feeding the shooter

    public static final double kRotateForIntakeSpeed         = 0.2; // negative for intake position
    public static final double kRotateForFeedingShooterSpeed = -0.35; // positive for shooter position

  } // end class Intake



  /**
   * a class with static constants and related defdinitions for the shooter
   */
  public static final class Shooter {

    public static final int kShooterLeftCanId    = 53;
    public static final int kShooterRightCanId   = 54;

    //public static final int kShooterRotatorCanId = 55; // if used

    public static PIDSettings shooterLeftPIDSettings = new PIDSettings(
      0.1, 1.0e-4, 1.0, 0.0, 0.0, -1.0, 1.0, "Shooter (Left)");

    public static PIDSettings shooterRightPIDSettings = new PIDSettings(
      0.1, 1.0e-4, 1.0, 0.0, 0.0, -1.0, 1.0, "Shooter (Right)");

    public static PIDSettings shooterRotatorPIDSettings = new PIDSettings(
      0.1, 1.0e-4, 1.0, 0.0, 0.0, -1.0, 1.0, "Shooter (Rotator)");
  
    // the following require tuning

    public static final double SHOOT_SPEED = 0.7;

    public static final double kSpeakerShotSpeed = 1.0; // positive for shooting
    public static final double kSpeakerShotAngle = 0.3; // use absolute encoder setting

    public static final double kDriveBySpeed     = 1.0; // positive for shooting
    public static final double kDriveByAngle     = 0.3; // use absolute encoder setting

    public static final double kAmpShotSpeed     = 1.0; // positive for shooting
    public static final double kAmpShotAngle     = 0.3; // use absolute encoder setting

    public static final double kTrapShotSpeed    = 1.0; // positive for shooting
    public static final double kTrapShotAngle    = 0.3; // use absolute encoder setting
    
    public static final double dTicks            = 10.0; // requires tuning

  } // end class Shooter



  /**
   * a class with static constants and related definitions for the swerve drive
   */
  public static final class Drive {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(25.5);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final double kModuleToCenterDistance = Math.sqrt(Math.pow(kWheelBase/2,2)+Math.pow(kTrackWidth/2,2));
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
    // Angular offsets of the modules relative to the chassis in radians
    /*
     * DON'T USE THESE! We have updated the zero offsets
     * in REV Hardware Client. These are now obsolete.
     */
    public static final double kFrontLeftChassisAngularOffset  =  0;
    public static final double kFrontRightChassisAngularOffset =  0;
    public static final double kBackLeftChassisAngularOffset   =  0;
    public static final double kBackRightChassisAngularOffset  =  0;

    // SPARK MAX CAN IDs

    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 22;
    public static final int kRearLeftDrivingCanId = 32;
    public static final int kRearRightDrivingCanId = 42;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kFrontRightTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 31;
    public static final int kRearRightTurningCanId = 41;

    // commented out (handled by overloading the gyro's methods)
    
    // public static final boolean kGyroReversed = false;

  } // end class Drive



  /**
   * a class with static constants and related definitions for a swerve module
   */
  public static final class Module {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotor.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

  } // end class Module



  /**
   * a class with static constants and related definitions for the operator interface, 
   * e.g., xbox controllers or joysticks
   */
  public static final class OI {

    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.4;

    public static final int JOYSTICK_X_AXIS = 1;
    public static final int JOYSTICK_Y_AXIS = 0;
    public static final int JOYSTICK_ROT_AXIS = 2;

    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorDeadband = 0.05;

    // operator xBox button mappings

    public static final int kLowerIntakeButton   = 0;
    public static final int kIntakeButton        = 0;
    public static final int kRaiseIntakeButton   = 0;
    public static final int kSpeakerShootButton  = 0;
    public static final int kAmpShootButton      = 0;
    public static final int kTrapShootButton     = 0;

    public static final int kStartShooterButton  = 0;
    public static final int kFeedShooterButton   = 0;

    public static final int kExtendClimberButton = 0;
    public static final int kClimbButton         = 0;
    public static final int kStopClimberButton    = 0;

    // driver xBox button mappings

    public static final int kGoSourceToSpeakerButton = 0;

  } // end class OI



  /**
   * a class with static constants and related definitions for autonomous (?)
   */
  public static final class Auto {

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  } // end class Auto



  /**
   * class with static constants and related definitions for NEO motors
   */
  public static final class NeoMotor {

    public static final double kFreeSpeedRpm = 5676;

  } // end class NeoMotor



  /**
   * class with static constants and related definitions for Falcon motors
   */
  public static final class FalconMotor {

    public static final int kTicksPerRevolution = 2048; // ticks per revolution
    public static final int kTimeoutMs          =   30; // in milliseconds

  } // end class FalconMotor


  /**
   * class with static constants and related definitions for sensors
   * e.g., limit switches, infrared or ultrasonic distance sensors
   */
  /*public static final class Sensor {

    // analog devices (Analog ports on RoboRIO)

    public static final UltrasonicDistanceSensor m_distance1 = new UltrasonicDistanceSensor(0,"US Sensor");
    public static final InfraredDistanceSensor   m_distance2 = new InfraredDistanceSensor  (1,"IR Sensor");

    // digital devices (DIO ports on RoboRIO)

    public static final LimitSwitch m_limit1 = new LimitSwitch(0, "cargo detected");

    public static enum Units {
      kMillimeters,
      kCentimeters,
      kMeters,
      kInches
    } // end enum Units

  } // end class Sensor*/



  /**
   * class with static constants and related definitions for the gyros,
   * e.g., the NAXZ2 or the FRC Gyro
   * 
   * @param identifier the nickname of the gyro in the SmartDashboard
   * @param reverseAngle if true, reverse the return values from getAngle() and getRate()
   * 
   */
  public static final class Gyro {

    public static final NavX2Gyro m_gyro  = new NavX2Gyro("NavX2 Gyro", true);
    //public static final FRCGyro   m_gyro2 = new FRCGyro  ("FRC Gyro", true  );

  } // end class Gyro



} // end class Constants
