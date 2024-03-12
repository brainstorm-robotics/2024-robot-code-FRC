// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType; // <<< INVESTIGATE <<<
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.Intake;

/**
 * The intake has one NEO motor to intake the notes. The positive motor direction
 * should be the intake operation, a negative motor direction shoud be the expel
 * operation for feeding the note to the shooter or for other operations, e.g.,
 * scoring in the amp or the trap.
 * 
 * A second NEO motor rotates the intake from the floor-intake position to the
 * shooter position and possibly other between positions, e.g., for shooting
 * from other positions or for scoring in the amp or the trap using the intake.
 * 
 * intake motor         (NEO, Mechanical Advantage: 3:1); 
 * intake rotator motor (NEO, Mechanical Advantage: 100:1).
 * 
 */
public class IntakeSubsystem extends SubsystemBase {

  // define motors

  private CANSparkMax mIntake;
  private CANSparkMax mIntakeRotator;

  private AbsoluteEncoder armEncoder;

  // define motor controllers

  //private SparkPIDController mIntakePID;
  //private SparkPIDController mIntakeRotatorPID;

  

  /** 
   * The intake has one intake motor and one intake rotor motor.
   * 
   * The intake motor speed should be negative when taking in a note and 
   * positive when ecpelling a note. 
   * 
   * The intake rotor motor speed should be negative when lowering to the 
   * intake position and positive when rotating towards a shooting position.
   * 
   * We should calibrate the intake so that we set optimum speeds for
   * taking in a note and for feeding a note to the shooter.
   * 
   * intake motor         (NEO, Mechanical Advantage 3:1); 
   * intake rotator motor (NEO, Mechanical Advantage 100:1);
   * 
   *  TESTING UNDER FIELD CONDITIONS IS NEEDED.
   */
  public IntakeSubsystem() {

    // initialize motor controllers

    mIntake = new CANSparkMax(Intake.kIntakeCanId, MotorType.kBrushless);
    mIntake.restoreFactoryDefaults();

    mIntakeRotator = new CANSparkMax(Intake.kIntakeRotatorCanId, MotorType.kBrushless);
    mIntakeRotator.restoreFactoryDefaults();

    armEncoder = mIntakeRotator.getAbsoluteEncoder();


    

    // initialize their PID controllers

    //mIntakePID = mIntake.getPIDController();
    //Intake.intakePIDSettings.setPIDControllerSettings(mIntakePID);

    //mIntakeRotatorPID = mIntakeRotator.getPIDController();
    //Intake.intakeRotatorPIDSettings.setPIDControllerSettings(mIntakeRotatorPID);

  } // end constructor IntakeSubsystem()

  public void setRollers(double speed){
    //mIntakePID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    //System.out.println(speed);
    mIntake.set(speed);
  }

  public void stopRollers() {
    // PID code to set the speed to feed the shooter

    //mIntakePID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    mIntake.set(0);

  } // end stopIntake()

  //returns true if arm properly rotated
  public boolean rotateArm(double speed){
    mIntakeRotator.setIdleMode(CANSparkBase.IdleMode.kBrake);
    //mIntakeRotatorPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    if(speed > 0){
      if(armEncoder.getPosition() < Intake.MAX_ARM_ROTATION){
        mIntakeRotator.set(speed);
        return true;
      }
    } else{
      if(armEncoder.getPosition() > Intake.MIN_ARM_ROTATION){
        mIntakeRotator.set(speed);
        return true;
      }
    }
    stopArm();
    return false;
  }

  public void stopArm() {
    // PID code to stop the intake rotor

    //mIntakeRotatorPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    mIntakeRotator.set(0);
    //mIntakeRotator.setIdleMode(CANSparkBase.IdleMode.kCoast);

  } // end stopIntakeRotator()



  @Override
  /**
   * This method will be called once per scheduler run
   */
  public void periodic() {

    // if SmartDashboard PID values have changed, update the PID controller

    Intake.intakePIDSettings.checkForChangedPID();
    Intake.intakeRotatorPIDSettings.checkForChangedPID();

    
    
  } // end periodic()

} // end class IntakeSubsystem
