// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.LocalTime;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev2mSensorSubsystem extends SubsystemBase {

  private double distance = 0.0;

  private Rev2mDistanceSensor sensor;

  /** 
   * create a new Rev2mSensorSubsystem
   */
  public Rev2mSensorSubsystem() {
    sensor = new Rev2mDistanceSensor(Port.kOnboard);
    this.startSensor();
  } // end constructor Rev2mSensorSubsystem



  @Override
  /**
   * called once per scheduler run
   */
  public void periodic() {

    this.getDistance();

  } // end periodic()



  public void startSensor() {
    sensor.setAutomaticMode(true);
    sensor.setEnabled(false);
  } // end start()


  /**
   * get the millimeter distance from the sensor 
   * 
   * @return the distance in mm
   */
  public double getDistance() {
    
    // if invalid range is reported, return a zero value

    if (! sensor.isRangeValid()) {
      return 0.0;
    } // end if

    // set the distance, update the SmartDashboard and return the distance

    distance = sensor.getRange(Unit.kMillimeters);
    sensor.getDistanceUnits();
    SmartDashboard.putNumber("REV 2m Sensor (mm) ", distance);
    System.out.print("\nREV 2m Sensor " + distance + " mm {" + 
      sensor.getTimestamp() + ", " + LocalTime.now() + "}\n");

    return distance;

  } // end getDistance()



  /**
   * stop the sensor
   */
  public void stopSensor() {

    sensor.setAutomaticMode(false);
    sensor.setEnabled(false);

  } // end stop()



}
