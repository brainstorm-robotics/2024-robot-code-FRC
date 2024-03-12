// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Be sure to orient the RoboRIO-2 so that the NavX-2 gyro is oriented in a
 * way consistent with the front of the robot and that the gyro is physically 
 * centered in the chassis.
 * (@see https://pdocs.kauailabs.com/navx-mxp/installation/roborio-installation/)
 */
public class NavX2Gyro extends AHRS {

  private String identifier;
  private boolean reverseAngle;



  /** 
   * create a new NavX2Gyro
   * 
   * @paran identifier the nickname used for this gyro in the SmartDashboard
   * @param reverseAngle wnen true, reverse / negate the return value from getAngle() and getRate()
  */
  public NavX2Gyro(String identifier, boolean reverseAngle) {

    super();
    this.identifier = identifier;
    this.reverseAngle = reverseAngle;
    
  } // end constructor NavX2Gyro()



  /**
   * Returns the angle, may be negated here depending on whether angles are CW or CCW
   * positive.
   * @return the angle
   */
  public double getAngle() {

    if (reverseAngle) {
      return -super.getAngle();
    } // end if

    return super.getAngle();

  } // end getAngle()



  /**
   * Get the rate, may be negated here depending on whether angles are CW or CCW
   * positive.
   * @return the rate
   */
  public double getRate() {

    if (reverseAngle) {
      return -super.getRate();
    } // end if

    return super.getRate();

  } // end getRate()



  public void updateSmartDashboard() {

    SmartDashboard.putNumber(identifier + "Roll",  this.getRoll());  /* when + roll  to  left */
    SmartDashboard.putNumber(identifier + "Pitch", this.getPitch()); /* when + tilt backwards */
    SmartDashboard.putNumber(identifier + "Yaw",   this.getYaw());   /* when + turn clockwise */

    SmartDashboard.putNumber(identifier + " ΔX", this.getDisplacementX()); // relative to starting field position
    SmartDashboard.putNumber(identifier + " ΔY", this.getDisplacementY()); // relative to starting field position
    SmartDashboard.putNumber(identifier + " ΔZ", this.getDisplacementZ()); // relative to starting field position

    SmartDashboard.putNumber(identifier + "Heading",  this.getAngle());
    SmartDashboard.putNumber(identifier + "Yaw Rate", this.getRate());

  } // end updateSmartDashboard()


  
} // end class NavX2Gyro
