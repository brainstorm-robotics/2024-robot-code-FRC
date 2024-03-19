package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimberDown extends Command{
  private ClimberSubsystem climber;
  private boolean state;

  public ClimberDown(ClimberSubsystem climber) {
    addRequirements(climber);

    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
    public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    state = climber.setState(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !state; //false if moving, true if stopped
  }
}
