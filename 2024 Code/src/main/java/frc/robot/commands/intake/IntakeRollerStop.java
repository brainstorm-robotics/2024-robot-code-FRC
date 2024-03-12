// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerStop extends Command {

  private IntakeSubsystem intake;
  private boolean finished = false;

  /** Creates a new Intake roller out. 
   * @param intake the subsytem that is used for intake
  */
  public IntakeRollerStop(IntakeSubsystem intake) {
    addRequirements(intake);

    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopRollers();
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
