// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.CommandTimer;

public class ShooterOutAmp extends Command {

  private boolean isTimed = false;
  private CommandTimer timer = null;

  private ShooterSubsystem shooter;

  private long duration;

  /** Creates a new Intake roller out. 
   * @param intake the subsytem that is used for intake
  */
  public ShooterOutAmp(ShooterSubsystem shooter) {
    addRequirements(shooter);

    this.shooter = shooter;
  }

  public ShooterOutAmp(ShooterSubsystem shooter, long duration) {
    this(shooter);
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
    shooter.shoot(Shooter.AMP_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0);
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
