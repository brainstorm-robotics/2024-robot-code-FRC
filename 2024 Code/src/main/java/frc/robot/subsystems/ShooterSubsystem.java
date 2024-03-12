// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
// import com.revrobotics.CANSparkMax.ControlType; <<< INVESTIGATE <<<
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import frc.robot.Constants.NeoMotor;

import frc.robot.Constants.Shooter;

/**
 * The shooter has two NEO Brushless motors for the shooter. Both should be 
 * turned at the same time and must counter-rotate with positive being shoot.
 * 
 * We should calibrate the shooter so that we set optimum shooter speeds for
 * shooting at different set points, e.g., from base of speaker, a short distance
 * back perhaps during a drive-by shoot, from further away. Consideration should
 * also be given to scoring in the amp and its required settings. Celt-X has
 * demonstrated that a specific shooting set-up may work for the trap.
 * 
 * The shooter may also have another NEO motor to rotate the elevation of the
 * shooter to optimum angles for different shooting angles.
 * 
 * left shooter motor      (NEO, Mechanical Advantage 1:1); 
 * right shooter motor     (NEO, Mechanical Advantage 1:1);
 * shooter elevation motor (NEO, Mechanical Advantage tba:1, if used).
 * 
 * Observations: The shooter consistently shoots notes. We discovered that the
 * notes we have have different masses. Two are 236g and they land in exactly
 * the same place. The other is 226g and this one has a slightly higher
 * trajectory. TESTING UNDER FIELD CONDITIONS IS NEEDED.
 */

public class ShooterSubsystem extends SubsystemBase {

  // define motors

  private CANSparkMax mShooterLeft;
  private CANSparkMax mShooterRight;
  // private CANSparkMax mShooterRotator;
  
  // define motor controllers

  //private SparkPIDController mShooterLeftPID;
  //private SparkPIDController mShooterRightPID;
  // private SparkMaxPIDController mShooterRotatorPID;

  

  /** 
   * Create a new Shooter
   */
  public ShooterSubsystem() {

    // initialize motor controllers

    mShooterLeft = new CANSparkMax(Shooter.kShooterLeftCanId, MotorType.kBrushless);
    mShooterLeft.restoreFactoryDefaults();

    mShooterRight = new CANSparkMax(Shooter.kShooterRightCanId, MotorType.kBrushless);
    mShooterRight.restoreFactoryDefaults();

    // mShooterRotator = new CANSparkMax(Shooter.kShooterRotatorCanId, MotorType.kBrushless);
    // mShooterRotator.restoreFactoryDefaults();
    
    // initialize their PID controllers

    //mShooterLeftPID = mShooterLeft.getPIDController();
    //Shooter.shooterLeftPIDSettings.setPIDControllerSettings(mShooterLeftPID);

    //mShooterRightPID = mShooterRight.getPIDController();
    //Shooter.shooterRightPIDSettings.setPIDControllerSettings(mShooterRightPID);

    // mShooterRotatorPID = mShooterRotator.getPIDController();
    // Shooter.shooterRotatorPIDSettings.setPIDControllerSettings(mShooterRotatorPID);
  
  } // end contructor ShooterSubsystem()



  /**
   * Start the shooter using PID for the left and right shooter motors.
   */
  public void shoot(double speed) {

    mShooterLeft.set(speed);
    mShooterRight.set(-speed);

    //mShooterLeftPID.setReference(Shooter.kSpeakerShotSpeed,  ControlType.kVelocity);
    //mShooterRightPID.setReference(Shooter.kSpeakerShotSpeed, ControlType.kVelocity);

  } // end startShooter()



/* the following not yet implemented ---------------------

  public void rotateShooterForBaseOfSpeakerShot() {

    // use the following to set the speaker shot angle (SparkMax)

    mShooterRotatorPID.setReference(Shooter.kSpeakerShotAngle, ControlType.kPosition);

  } // end rotateShooterForBaseOfSpeakerShot()

---------------------------------------------------------- */



  @Override
  /**
   * This method will be called once per scheduler run
   */
  public void periodic() {
    
    // if SmartDashboard PID values have changed, update the PID controller

    Shooter.shooterLeftPIDSettings.   checkForChangedPID();
    Shooter.shooterRightPIDSettings.  checkForChangedPID();
    Shooter.shooterRotatorPIDSettings.checkForChangedPID();
   
  } // end periodic()



} // end class ShooterSubsystem
