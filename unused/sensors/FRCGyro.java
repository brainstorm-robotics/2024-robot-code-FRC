// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The FRC Gyro orientation is different from that of the NavX-2 however
 * this really doesn't matter. When initializing this gyro (at the same
 * time as initializing the NavX-2) the orientation is "zeroed". Since
 * the method calls to the gyro are limited to getAngle() and getRate(),
 * the returned values relative to the initialization points should be the
 * same as the values returned by the NavX-2.
 * 
 * If these assumtions are erroneous then a bias may be added to the angle
 * before returning the angle using getAngle().
 */
public class FRCGyro extends ADXRS450_Gyro {

  private String identifier;
  private boolean reverseAngle;
  
  public FRCGyro(String identifier, boolean reverseAngle) {
    
    super();
    this.identifier   = identifier;
    this.reverseAngle = reverseAngle;

  } // end constructor FRCGyro()



  /**
   * Get the angle, may be negated here depending on whether angles are CW or CCW 
   * positive. May need to add a bias to align angle with the NavX-2.
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
  
  
  
  /**
   * update the SmartDashboard
   */
  public void updateSmartDashboard() {

    SmartDashboard.putNumber(identifier + " heading",  this.getAngle());
    SmartDashboard.putNumber(identifier + " yaw rate", this.getRate());

  } // end updateSmartDashboard()

} // end class FRCGyro
