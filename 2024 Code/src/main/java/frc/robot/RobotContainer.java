// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import frc.robot.Constants.Sensor;
import frc.robot.Constants.Gyro;
import frc.robot.Constants.OI;
import frc.robot.commands.Brake;
import frc.robot.commands.commandGroups.IntakeCycle;
import frc.robot.commands.commandGroups.IntakeIn;
import frc.robot.commands.commandGroups.Shoot;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.ShooterOut;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.sensors.FRCGyro;
//import frc.robot.subsystems.sensors.InfraredDistanceSensor;
//import frc.robot.subsystems.sensors.LimitSwitch;
import frc.robot.subsystems.sensors.NavX2Gyro;
//import frc.robot.subsystems.sensors.UltrasonicDistanceSensor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // gyros

    private final NavX2Gyro gyro  = Gyro.m_gyro;  // replacement gyro
//  private final FRCGyro   gyro2 = Gyro.m_gyro2; // redundant/back-up gyro
  
  // The robot's subsystems

  private final DriveSubsystem   m_robotDrive = new DriveSubsystem();

  private final IntakeSubsystem intake = new IntakeSubsystem();

  private final ShooterSubsystem shooter = new ShooterSubsystem();
  //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(gyro);
  //private final IntakeSubsystem  intakeSubsystem  = new IntakeSubsystem();
  //private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    
  // analog distance sensors

  //private final UltrasonicDistanceSensor m_distance1 = Sensor.m_distance1; // example
  //private final InfraredDistanceSensor   m_distance2 = Sensor.m_distance2; // example
  //private final LimitSwitch              m_limit1    = Sensor.m_limit1;    // example

  // chooser for Pathplanner

  private final SendableChooser<Command> autoChooser;

  // The driver and operator controllers
  
  Joystick m_driverController   = new Joystick(OI.kDriverControllerPort);
  //XboxController m_operatorController = new XboxController(OI.kOperatorControllerPort);



  /**
   * construct a RobotContainer, containing subsystems, OI devices, and commands
   */
  public RobotContainer() {

    // Configure the button bindings

    configureButtonBindings();

    // Configure default commands

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getRawAxis(OI.JOYSTICK_X_AXIS),  OI.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRawAxis(OI.JOYSTICK_Y_AXIS),  OI.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRawAxis(OI.JOYSTICK_ROT_AXIS), OI.kDriveDeadband),
            true, true),
          m_robotDrive).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      );

      NamedCommands.registerCommand("brake", new Brake(m_robotDrive));

    // calibrate the gyros

    // gyro .calibrate(); // NavX2 Gyro <<< INVESTIGATE <<<
    // gyro2.calibrate(); // FRC Gyro

    // *** Start of code for Pathplanner chooser

      // Build an auto chooser that uses Commands.none() as the default option

      autoChooser = AutoBuilder.buildAutoChooser();

      // Another option that allows you to specify the default auto by its name
      // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

      // put the choices on the SmartDashboard
    
      SmartDashboard.putData("Auto Chooser", autoChooser);

    // *** End of code for Pathplanner chooser

  } // end constructor RobotContainer()



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /*new JoystickButton(m_driverController, Button.kR1.value)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive)
    );*/

    new JoystickButton(m_driverController, 1).onTrue(
      new ShooterOut(shooter, 1000).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    new JoystickButton(m_driverController, 3).onTrue(//roll the things in
      new IntakeRollerIn(intake, 1000).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    new JoystickButton(m_driverController, 4).onTrue(//roll the things out
      new IntakeRollerOut(intake, 1000).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    new JoystickButton(m_driverController, 2).onTrue(//roll the things stop
      new IntakeRollerStop(intake).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    new JoystickButton(m_driverController, 5).onTrue(
      new IntakeArmDown(intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );

    new JoystickButton(m_driverController, 6).onTrue(
      new IntakeArmUp(intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );

    new JoystickButton(m_driverController, 7).onTrue(
      new IntakeIn(intake, 1500)
    );

    new JoystickButton(m_driverController, 8).onTrue(
      new IntakeCycle(intake, 1500)
    );

    new JoystickButton(m_driverController, 9).onTrue(
      new Shoot(intake, shooter)
    );




    // use m_driverController or m_operatorController

    // Each mapping is two lines of code. The first line is fine and connects
    // to a button. The second line needs to be connected to a command

    // operator and driver button-to-command mappings

    // lower the intake

    // new JoystickButton(m_operatorController, OI.kLowerIntakeButton)
    //    .whileActiveOnce(new ElevatorPIDCmd(intakeSubsystem, OI.ElevatorConstants.kRaisedPosition));
    
    // lower the intake if necessary and take in a note. On successful intake, rotate
    // the intake to shooter position

    // new JoystickButton(m_operatorController, OI.kIntakeButton)
    //     .whileActiveOnce(new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kLoweredPosition));

    // raise the intake to the shooter position, available on both controllers

    // new JoystickButton(m_operatorController, OI.kRaiseIntakeButton)
    //     .whileActiveOnce(new ElevatorJoystickCmd(elevatorSubsystem, ElevatorConstants.kJoystickMaxSpeed));
    // new JoystickButton(m_driverController, OI.kRaiseIntakeButton)
    //     .whileActiveOnce(new ElevatorJoystickCmd(elevatorSubsystem, ElevatorConstants.kJoystickMaxSpeed));

    // move the intake to the shooter position if necessary. Start the shooter motors.
    // when shooter motors are at speed, feed the note to the shooter. When shot is
    // done, stop the shooter motors and stop the intake motor

    // new JoystickButton(m_operatorController, OI.kSpeakerShootButton)
    //     .whileActiveOnce(new ElevatorJoystickCmd(elevatorSubsystem, -ElevatorConstants.kJoystickMaxSpeed));
    
    // rotate the intake to the amp shooting angle
    // shoot the note from the intake at amp speed

    // an alternative algorithm:
    //
    // rotate the intake to shooter position. Start shooter motors at amp speed.
    // Feed the note from the intake to the shooter. When shot is done, stop
    // shooter motors and intake motor.

    // new JoystickButton(m_operatorController, OI.kAmpShootButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // rotate the intake to the trap shooting angle
    // shoot the note from the intake at trap speed

    // an alternative algorithm:
    //
    // rotate the intake to shooter position. Start shooter motors at trap speed.
    // Feed the note from the intake to the shooter. When shot is done, stop
    // shooter motors and intake motor.

    // new JoystickButton(m_operatorController, OI.kTrapShootButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // start the shooter motor to xxx speed (maybe pass a parameter?)

    // new JoystickButton(m_operatorController, OI.kStartShooterButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // stop the shooter motor.

    // new JoystickButton(m_operatorController, OI.kFeedShooterButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // set intake motor to speed for feeding the shooter

    // new JoystickButton(m_operatorController, OI.kFeedShooterButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // extend the climber.

    // new JoystickButton(m_operatorController, OI.kExtendClimberButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // retract the climber

    // new JoystickButton(m_operatorController, OI.kClimbButton)
    //     .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));

    // stop the climber

    // new JoystickButton(m_operatorController, OI.kStopClimberButton)
    //     .whileActiveOnce(new StopClimberCommand(climberSubsystem, false));     

  } // end configureButtonBindings()



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return autoChooser.getSelected();
    return new PathPlannerAuto("straight");

  } // end getAutonomousCommand()



  /**
   * access to the main gyro
   * 
   * @return
   */
  public NavX2Gyro getGyro() {
    return gyro;
  } // end getGyro;



} // end class RobotContainer
