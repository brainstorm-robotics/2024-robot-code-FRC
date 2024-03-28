package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.CommandTimer;

public class ClimberUp extends Command{
  private ClimberSubsystem climber;
  private long duration;
  private CommandTimer timer;

  public ClimberUp(ClimberSubsystem climber, long duration) {
    addRequirements(climber);
    this.duration = duration;
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
    public void initialize() {
        timer = new CommandTimer(duration);
        climber.setBrakes(false);
    }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setState(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if(timer.isComplete()) {
        return true;   
    } */
    return false;
  }
}
