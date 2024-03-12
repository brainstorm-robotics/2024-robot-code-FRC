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
public class UltrasonicDistanceSensor extends AnalogInput {

  private String identifier;



  public UltrasonicDistanceSensor(int channel, String identifier) {

    super(1);
    this.identifier = identifier;

  } // end constructor UltrasonicDistanceSensor()



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
    double millimeters = 1024.0 * voltage; // from sensor documentation

    switch (units) {
      case kCentimeters: return millimeters / 10.0;
      case kMeters:      return millimeters / 1000.0;
      case kInches:      return millimeters / 25.4;
      default:           return millimeters;
    } // end switch (units)

  } // end getDistance()


  
 /**
  * update the dashboard with the current voltage and distance
  */
  public void updateSmartDashboard() {

    SmartDashboard.putNumber(identifier + "Voltage (V)",  this.getVoltage());
    SmartDashboard.putNumber(identifier + "Distance (m)", this.getDistance(Units.kMeters));

  } // end updateSmartDashboard()



} // end cass UltrasonicSensor
