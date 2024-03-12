// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.Constants.FalconMotor;
// import frc.robot.Constants.Climber;

/** 
 * Add your docs here. 
 */
public class MyTalonFX extends TalonFX {

   public MyTalonFX(int can) {
    super(can);
  } // end constructor MyTalonFX()

  /**
   * set the key parameters for a TalonFX
   * 
   * @param deadBand the motor's deadband as a decimal, e.g. 0.04 is 4%
   * @param inverted whether the motor direction is inverted
   * @param velocity velocity in ticks per 100ms (2048 ticks per revolution)
   * @param acceleration acceleration in ticks per 100ms per s
   * @param nominalForward forward power to hold motor in position, e.g., to overcome gravity
   * @param nominalBackward forward power to hold motor in position, e.g., to overcome gravity
   */
  public void initialize(double deadBand, boolean inverted,
                         double velocity, double acceleration,
                         double nominalForward, double nominalBackward) {

  //  configFactoryDefault();
  //   configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconMotor.kTimeoutMs);
  //   setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, FalconMotor.kTimeoutMs);

    // initialize the TalonFX motor controller settings
    
  //  configNeutralDeadband(     deadBand,        FalconMotor.kTimeoutMs);
    setInverted(               inverted);
	//   configMotionCruiseVelocity(velocity,        FalconMotor.kTimeoutMs);
	//   configMotionAcceleration(  acceleration,    FalconMotor.kTimeoutMs);    
  //   configNominalOutputForward(nominalForward,  FalconMotor.kTimeoutMs);
  //   configNominalOutputReverse(nominalBackward, FalconMotor.kTimeoutMs);
  
  } // end initialize()

} // end class MyTalonFX
