// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.time.LocalTime;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/** 
 * use this class as a convenient container for PID settings and PID manipulation
 * to cut down considerably on "donkey work" coding of repeated code blocks. By
 * linking directly to the PID controller, all the heavy lifting is done here.
 */
public class PIDSettings {

  // define PID controller and PID variables

  private SparkPIDController controller;
  
  private double p;
  private double i;
  private double d;
  private double iz;
  private double ff;
  private double min;
  private double max;
  private String id;



  /**
   * construct a PIDSettings object with the given PID settings
   * 
   * @param p the P value
   * @param i the I value
   * @param d the D value
   * @param iz the I zone value
   * @param ff the forward feed value
   * @param min the minimum output
   * @param max the maximum output
   * @param id an identifier used when accessing PID values in SmartDashboard
   */
  public PIDSettings(double p, double i, double d, double iz, double ff, 
                     double min, double max, String id) {
    
    this.p   = p;
    this.i   = i;
    this.d   = d;
    this.iz  = iz;
    this.ff  = ff;
    this.min = min;
    this.max = max;
    this.id  = id;

    SmartDashboard.putNumber(this.id+" p",   this.p);
    SmartDashboard.putNumber(this.id+" i",   this.i);
    SmartDashboard.putNumber(this.id+" d",   this.d);
    SmartDashboard.putNumber(this.id+" iz",  this.iz);
    SmartDashboard.putNumber(this.id+" ff",  this.ff);
    SmartDashboard.putNumber(this.id+" min", this.min);
    SmartDashboard.putNumber(this.id+" max", this.max);
   
    logPIDSettings();

  } // end constructor PIDSettings()



  // accessors for PID settings

  public double getP()   { return this.p;  } // end getP()
  public double getI()   { return this.i;  } // end getI()
  public double getD()   { return this.d;  } // end getD()
  public double getIz()  { return this.iz; } // end getIz()
  public double getFF()  { return this.ff; } // end getFF()
  public double getMin() { return this.min;} // end getMin()
  public double getMax() { return this.max;} // end getMax()

    

  /**
   * apply the PID settings directly to the PID controller 
   * 
   * @param the PID controller
   */
  public void setPIDControllerSettings(SparkPIDController controller) {

    // remember the PID controller for future use

    this.controller = controller;

    // assign PID values to the PID controller
    
    controller.setP(p);
    controller.setI(i);
    controller.setD(d);
    controller.setIZone(iz);
    controller.setFF(ff);
    controller.setOutputRange(min, max);

  } // end setPIDController()



  /**
   * if SmartDashboard PID values have changed, update the PID controller
   * 
   *  @param id an identifier for these PID settings
   */
  public void checkForChangedPID() {

    boolean changeFound = false;

    // get the PID values from the SmartDashboard

    double p   = SmartDashboard.getNumber(this.id+" P",   this.p);
    double i   = SmartDashboard.getNumber(this.id+" I",   this.i);
    double d   = SmartDashboard.getNumber(this.id+" D",   this.d);
    double iz  = SmartDashboard.getNumber(this.id+" Iz",  this.iz);
    double ff  = SmartDashboard.getNumber(this.id+" FF",  this.ff);
    double min = SmartDashboard.getNumber(this.id+" Min", this.min);
    double max = SmartDashboard.getNumber(this.id+" Max", this.max);

    // if a PID setting has changed update the controller
    
    if (this.p  != p)  { controller.setP(p);      this.p = p;   changeFound = true;}
    if (this.i  != i)  { controller.setI(i);      this.i = i;   changeFound = true;}
    if (this.d  != d)  { controller.setD(d);      this.d = d;   changeFound = true;}
    if (this.iz != iz) { controller.setIZone(iz); this.iz = iz; changeFound = true;}
    if (this.ff != ff) { controller.setFF(ff);    this.ff = ff; changeFound = true;}
  
    if ((this.min != min) || (this.max != max)) {
      controller.setOutputRange(min, max);
      changeFound = true;
    } // end if

    if (changeFound) {
      logPIDSettings();
    } // end if

  } // end checkForChangedPID()



  /**
   * write the PID settings to the system log
   */
  private void logPIDSettings() {
          
    System.out.print("/n" + this.id + " PID Settings: " +
                     this.p   + ", " + this.i   + ", " + this.d + ", " + 
                     this.iz  + ", " + this.ff  + ", " + 
                     this.min + ", " + this.max + " { " + 
                     LocalTime.now() + "}/n");

  } // end logPIDSettings()

} // end class PIDSettings
