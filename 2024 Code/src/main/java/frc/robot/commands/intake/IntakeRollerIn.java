// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.CommandTimer;

public class IntakeRollerIn extends Command {

  private IntakeSubsystem intake;

  private boolean isTimed = false;
  private CommandTimer timer = null;

  private long duration;

  /** Creates a new Intake roller in. 
   * @param intake the subsytem that is used for intake
  */
  public IntakeRollerIn(IntakeSubsystem intake) {
    addRequirements(intake);

    this.intake = intake;
  }

  public IntakeRollerIn(IntakeSubsystem intake, long duration) {
    this(intake);
    this.isTimed = true;
    this.duration = duration;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new CommandTimer(duration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRollers(Intake.kIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollers(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isTimed) {
      if(timer.isComplete()) {
        return true;
      } 
    }
    return false;
  }
}
