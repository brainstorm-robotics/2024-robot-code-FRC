package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.CommandTimer;

public class Brake extends Command{
    DriveSubsystem drive;
    private CommandTimer timer;

    public Brake(DriveSubsystem drive){
        addRequirements(drive);
        this.drive = drive;
    }
    
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new CommandTimer(5000);//change later
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setX();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.isComplete();
  }
}
