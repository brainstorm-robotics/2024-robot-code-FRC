// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// source code is from Rev Robotics
//
// Changed the sensor from ADIS16470 IMU to NavX2
// the method calls seem to be identical for:
//    getRate(), 
//    getAngle(), 
//    reset() and 
//    their constructors
// so no code changes other than replacing the class for m_gyro, were necessary.

package frc.robot.subsystems;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.utils.SwerveUtils;

import frc.robot.Constants.Drive;
import frc.robot.Constants.Gyro;
// import frc.robot.subsystems.sensors.FRCGyro;
import frc.robot.subsystems.sensors.NavX2Gyro;



public class DriveSubsystem extends SubsystemBase implements BooleanSupplier {

  // Create MAXSwerveModules

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      Drive.kFrontLeftDrivingCanId,
      Drive.kFrontLeftTurningCanId,
      Drive.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      Drive.kFrontRightDrivingCanId,
      Drive.kFrontRightTurningCanId,
      Drive.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      Drive.kRearLeftDrivingCanId,
      Drive.kRearLeftTurningCanId,
      Drive.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      Drive.kRearRightDrivingCanId,
      Drive.kRearRightTurningCanId,
      Drive.kBackRightChassisAngularOffset);

  // The gyro sensor

  // *** Replaced the ADIS16470_IMU gyro with the NavX2 ***

  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final NavX2Gyro gyro  = Gyro.m_gyro;  // primary gyro
  // private final FRCGyro   gyro2 = Gyro.m_gyro2; // secondary gyro
  
  // Slew rate filter variables for controlling lateral acceleration
  double maxSpeed = 0;
  double maxAccel = 0;
  long lastFrame = 0;
  double lastAngle = 0;



  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Drive.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Drive.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      Drive.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle()), // *** verify sensor code ***
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });



  /** 
   * create a new DriveSubsystem
   */
  public DriveSubsystem() {
    lastFrame = System.currentTimeMillis();
    lastAngle = gyro.getAngle();

    // *** code added for Path Planner ***

    // Configure the AutoBuilder

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        // this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::resetOdometry,
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1, 0.0, 0), // Translation PID constants
            new PIDConstants(1, 0.0, 0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            Drive.kModuleToCenterDistance, // Radius in meters of 2 robot using l^2 +w^2 both in inches
        new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this,
        this // Reference to this subsystem to set requirements
    );

    // *** end of code added for Path Planner ***
  
  } // end constructor DriveSubsystem()


  /*
  /**
   * This method is required to implement the BooleanSupplier for the
   * PathPlanner's AutoBuilder. The value returned should be obtained
   * from the SmartDashboard.
   * 
   * @return true if Blue Alliance, false otherwise
   */
  public boolean getAsBoolean() {

    boolean onRed = SmartDashboard.getBoolean("Red Alliance",true);

    // report alliance colour in the System Log

    String allianceString;

    if (onRed){
      allianceString = "Red";
    } // end if
    else {
      allianceString = "Blue";
    } // end else

    System.out.println("/n*** " + allianceString + " alliance ***/n/n");

    return onRed;
    
  } // end getAsBoolean()


  
  @Override
  public void periodic() {

    // Update the odometry in the periodic block

    m_odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()), // *** verify sensor code ***
      new SwerveModulePosition[] {
        m_frontLeft. getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.  getPosition(),
        m_rearRight. getPosition()
      }
    );
        
  } // end periodic()



  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }



  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()), // *** verify sensor code ***
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
    pose);
  }



  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of rotation of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Drive.kDirectionSlewRate / m_currentTranslationMag);
      } // end if 
      else {
        directionSlewRate = 1000.0; //some high number that means the slew rate is effectively instantaneous
      } // end else

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } // end if

      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } //end if
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        } // end else
      } // end else if

      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      } // end else

      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        m_currentRotation = rot;
    } // end else

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Drive.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Drive.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Drive.kMaxAngularSpeed;

    //System.out.println("is field relative? " + fieldRelative);

    var swerveModuleStates = Drive.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, 
              Rotation2d.fromDegrees(gyro.getAngle())) // *** verify sensor code ***
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Drive.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    long deltaTime = System.currentTimeMillis() - lastFrame;
    lastFrame = System.currentTimeMillis();
    double deltaAngle = gyro.getAngle() - lastAngle;
    lastAngle = gyro.getAngle();
    double speed = Math.abs(deltaAngle/deltaTime) * 1000;
    double accel = (speed/deltaTime) * 1000;

    if(speed > maxSpeed){
      maxSpeed = speed;
      //System.out.println("vel: " + maxSpeed);
    }

    if(accel > maxAccel){
      maxAccel = accel;
      //System.out.println("accel: " + maxAccel);
    }
  } // end drive()



  // *** start of code added to support Pathplanner ***

  public void driveRobotRelative(ChassisSpeeds speeds){

    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,
               false,false);

  } // end driveRobotRelative()


  
  public ChassisSpeeds getRobotRelativeSpeeds(){

    return Drive.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                  m_frontRight.getState(),
                                                  m_rearLeft.getState(),
                                                  m_rearRight.getState());

  } // end getRobotRelativeSpeeds()

  // *** end of code added to support Pathplanner ***



  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {

    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

  } // end setX()



  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drive.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

  } // end setModuleStates()



  /** 
   * reset the drive encoders to currently read a position of 0
   */
  public void resetEncoders() {

    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();

  } // end resetEncoders()



  /** 
   * zeroes the heading of the robot
   */
  public void zeroHeading() {

    gyro.reset();
    // m_gyro2.reset();

  } // end zeroHeading()



  /**
   * 
   * returns the heading of the robot
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {

    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();

  } // end getHeading()



  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {

    // return m_gyro.getRate() * (Drive.kGyroReversed ? -1.0 : 1.0);
    return gyro.getRate(); // gyro handles the negation / reversal

  } // end getTurnRate()



} // end class DriveSubsystem
