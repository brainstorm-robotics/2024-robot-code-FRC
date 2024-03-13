package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.CommandTimer;

public class WaitMillis extends Command{
    private CommandTimer timer;

    private long duration;

    public WaitMillis(long duration){
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

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.isComplete();
    }
}
