// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Sensor.Units;



/**
 * This models the MB1043-000 Ultrasonic Distance Sensor
 */
public class InfraredDistanceSensor extends AnalogInput {

  private String identifier;



  /** 
   * constructor to create a new UltrasonicDistanceSensor. 
   */
  public InfraredDistanceSensor(int channel, String identifier) {
    super(channel);
    this.identifier = identifier;
  } // end constructor InfraredDistanceSensor()



  /**
   * determines the distance from the sensor in meters
   * @returns the distance in meters
   */
   public double getDistance() {

     return this.getDistance(Units.kMeters);

   } // end getDistance()



  /**
   * determines the distance from the sensor in the required units
   * @param units one of millimeters, centimeters, meters or inches
   * @returns the distance in the specified units
   */
  public double getDistance(Units units) {

    double voltage = getVoltage();
    double millimeters = 237 / (voltage - 0.128) + 1.85;  // from Desmos regression model, 
                                                          // see https://www.desmos.com/calculator/psvong22yf

    switch (units) {
      case kCentimeters: return millimeters / 10.0;
      case kMeters:      return millimeters / 1000.0;
      case kInches:      return millimeters / 25.4;
      default:           return millimeters;
    } // end switch

  } // end getDistance()



 /**
  * update the dashboard with the current voltage and distance
  */
  public void updateSmartDashboard() {

    SmartDashboard.putNumber(identifier + " voltage (V)",  this.getVoltage());
    SmartDashboard.putNumber(identifier + " distance (m)", this.getDistance(Units.kMeters));

  } // end updateSmartDashboard()


  
} // end class InfraredSensor
