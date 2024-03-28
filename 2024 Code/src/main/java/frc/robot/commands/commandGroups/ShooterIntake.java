package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitMillis;
import frc.robot.commands.intake.IntakeRollerIn;
import frc.robot.commands.intake.IntakeRollerOut;
import frc.robot.commands.shooter.ShooterIn;
import frc.robot.commands.shooter.ShooterOut;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIntake extends ParallelCommandGroup{
    public ShooterIntake(IntakeSubsystem intake, ShooterSubsystem shooter){
        super(//deadline, commands...
            new IntakeRollerIn(intake, 500),
            new ShooterIn(shooter, 500)
        );
    }
}
