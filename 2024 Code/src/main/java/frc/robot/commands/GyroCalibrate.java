package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.sensors.NavX2Gyro;
import frc.utils.CommandTimer;

public class GyroCalibrate extends Command{
  NavX2Gyro gyro;
  public GyroCalibrate(NavX2Gyro gyro){
    this.gyro = gyro;
  }
    
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.zeroYaw();
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
    return true;
  }
}
