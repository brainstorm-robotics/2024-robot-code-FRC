package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitMillis;
import frc.robot.commands.intake.IntakeRollerOut;
import frc.robot.commands.shooter.ShooterOut;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends ParallelDeadlineGroup{
    public Shoot(IntakeSubsystem intake, ShooterSubsystem shooter){
        super(//deadline, commands...
            new SequentialCommandGroup(
                new WaitMillis(1000),
                new IntakeRollerOut(intake, 500)
            ),
            new ShooterOut(shooter)
        );
    }
}
