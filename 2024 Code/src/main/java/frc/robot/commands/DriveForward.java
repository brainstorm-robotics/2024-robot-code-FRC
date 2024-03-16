package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.CommandTimer;

public class DriveForward extends Command{
    DriveSubsystem drive;
    CommandTimer timer;

    public DriveForward(DriveSubsystem drive){
        this.drive = drive;
    }

    @Override
  public void initialize() {
    timer = new CommandTimer(1000);//change later
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveRobotRelative(new ChassisSpeeds(0.25, 0, 0));
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
