// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The LimitSwitch class may be used for all mechanical, optical or ultrasonic switches. The advantage
 * of using this class rather than directly using DigitalInput is that here the status of the LimitSwitch
 * is logged in the SmartDashboard. In addition to the DIO port number and to distinguish among LimitSwitch 
 * objects, an identifier must be provided in the LimitSwitch constructor.
 */

public class LimitSwitch extends DigitalInput {

  private String identifier;



  /** 
  * creates a new LimitSwitch
  * @param channel the DIO port number on the RoboRIO
  * @param identifier the label to be used for the status of this LimitSwitch in the SmartDashboard
  * @returns true if the limit switch was triggered, false otherwise
  */
  public LimitSwitch(int channel, String identifier) {

    super(channel);
    this.identifier = identifier;

  } // end constructor LimitSwitch()



 /**
  * update the dashboard with the current voltage and distance
  */
  public void updateSmartDashboard() {

    SmartDashboard.putBoolean(identifier, this.get());

  } // end updateSmartDashboard()
 


} // end class LimitSwitch
