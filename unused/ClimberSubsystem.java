// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.Constants.FalconMotor;
import frc.robot.Constants.Climber;

import frc.robot.subsystems.sensors.NavX2Gyro;

import frc.utils.MyTalonFX;

/**
 * The climber uses two Falcon-500 motors, one for a telescoping tube on each
 * side of the robot. This subsystem should include code to keep the robot level
 * as it climbs. The gyro may be used - check the roll using the method
 * getRoll(). When the return value is positive, there is a roll to left, 
 * negative is roll to the right. Should have a level tolerance similar to
 * deadbanding for motor speeds.
 * 
 * Be sure to calibrate and set both limits (extended, retracted) so that code
 * can keep motor rotations between these settings.
 * 
 * left climber motor  (Falcon, Mechanical Advantage 15:1);
 * right climber motor (Falcon, Mechanical Advantage 15:1);
 */

public class ClimberSubsystem extends SubsystemBase {
    
  // Define motors

  private MyTalonFX mClimberLeft;
  private MyTalonFX mClimberRight;

  /** 
   * 
   * CAUTION: NEEDS A SERIOUS LOOK - Falcon Code now uses Phoenix 6 framework and many
   * of the methods are very different so need to figure out how to use the new API for
   * Falcon 500 TalonFX.
   * 
   * The climber has two Falcon motors, one on each side, to operate independent
   * telescoping climbers based on springs (to extend) and a winch (to retract).
   * Both should be turned on at the same time. Because climbing is on a chain,
   * the climber on the left and on the right must operate independently. The 
   * NavX-2 gyro may be consulted to determine whether the robot is hanging level
   * and, if not, then one side may be retracted more than the other.
   * 
   * We should calibrate the climber motors to set limits with the retracted position
   * being the zero.
   * 
   * left climber motor  (Falcon, Mechanical Advantage tba:1); 
   * right climber motor (Falcon, Mechanical Advantage tba:1);
   * 
   * TESTING UNDER FIELD CONDITIONS IS NEEDED.
   */
  public ClimberSubsystem(NavX2Gyro gyro) {

    // initialize motors

    mClimberLeft = new MyTalonFX(Climber.kClimberLeftCanId);
    mClimberLeft.initialize(Climber.kDeadband, Climber.kInvertedLeft,
                            Climber.kVelocity, Climber.kAcceleration,
                            Climber.kNominalBackward, Climber.kNominalForward);
  
    mClimberRight = new MyTalonFX(Climber.kClimberRightCanId);
    mClimberRight.initialize(Climber.kDeadband, Climber.kInvertedRight,
                             Climber.kVelocity, Climber.kAcceleration,
                             Climber.kNominalBackward, Climber.kNominalForward);

    // zero the sensors

//     mClimberLeft. getSensorCollection().
//       setIntegratedSensorPosition(0.0, FalconMotor.kTimeoutMs);

//     mClimberRight.getSensorCollection().
//       setIntegratedSensorPosition(0.0, FalconMotor.kTimeoutMs);
        
  } // end contructor ClimberSubsystem()



  /*
   * climb method, adjusts the robot if it is hanging awkwardly on the chain
   * 
   * Calculations to detemine the velocity and number of turns of the motor to
   * climb are available at:
   * https://sixnationspolytechnic-my.sharepoint.com/:x:/g/personal/peter_gehbauer_snpsteam_com/ERpElEVt8AZMkXdbAop_qHUB7fJerqqlZLFKiYfWivSySw?e=csPPaI)
   * 
   * @param gyro the NavX2 gyro to use to test whether the robot is level
   */
  public void climb(NavX2Gyro gyro) {

    // to climb, set the motors to return to the starting position
    // but check whether the robot is level or not using the NavX-2 gyro

    // ATTENTION: verify whether to use NavX-2's getRoll(), getPitch()
    // or getYaw() method, given the NavX-2 orientation

    if (gyro.getRoll() > Climber.kAngleTolerance /* degrees */) {
      
      // robot is high on the right side, do something about it!

  //    mClimberRight.set(ControlMode.Velocity, 0.0); // stop the right motor and
  //    mClimberLeft.set(ControlMode.Position, 0.0);  // allow left to catch up

    } // end if

    else if (gyro.getRoll() < -Climber.kAngleTolerance /* degrees */)  {
      
      // robot is high on the left side, do something about it!

  //    mClimberLeft.set(ControlMode.Velocity, 0.0);  // stop the left motor and
  //    mClimberRight.set(ControlMode.Position, 0.0); // allow right to catch up

    } // end else if

  //  else if ((Math.abs(mClimberLeft. getSelectedSensorPosition()) < Climber.kClimbTolerance) ||
  //           (Math.abs(mClimberRight.getSelectedSensorPosition()) < Climber.kClimbTolerance)) {
  //
  //     // robot is within tolerance of being level and at least one side 
  //     // of the climber has reached the top so stop
  //
  //     mClimberLeft.set( ControlMode.Velocity, 0.0); // stop left motor
  //     mClimberRight.set(ControlMode.Velocity, 0.0); // stop right motor
  //
  //  } // end else if
  //
  //   else {
  //
  //    // climb equally on both sides
  //
  //     mClimberLeft.set(ControlMode.Position, 0.0);
  //     mClimberRight.set(ControlMode.Position, 0.0);  
  //
  //   } // end if

  } // end climb()



  public void extend() {

    // extend the arm

    // Motion Magic

    double targetPos = Climber.kWinchRevolutions * Climber.kMechanicalAdvantage * FalconMotor.kTicksPerRevolution;
		
//    mClimberLeft.set(ControlMode.Position,  targetPos);
//    mClimberRight.set(ControlMode.Position, targetPos);

  } // end extend()



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  } // end periodic()

} // end class ClimberSubsystem
