package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForward;
import frc.robot.commands.intake.IntakeArmDown;
import frc.robot.commands.intake.IntakeRollerIn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInDrive extends SequentialCommandGroup{
    public IntakeInDrive(IntakeSubsystem intake, DriveSubsystem drive){
        this(intake, drive, 1000);
    }

    public IntakeInDrive(IntakeSubsystem intake, DriveSubsystem drive, long duration){
        addCommands(
            new IntakeArmDown(intake),
            new ParallelCommandGroup(
                new IntakeRollerIn(intake, duration),
                new DriveForward(drive, duration)
            )
        );
    }
}
