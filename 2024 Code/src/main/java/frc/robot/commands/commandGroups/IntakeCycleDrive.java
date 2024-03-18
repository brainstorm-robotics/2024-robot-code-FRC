package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeArmUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCycleDrive extends SequentialCommandGroup{
    public IntakeCycleDrive(IntakeSubsystem intake, DriveSubsystem drive){
        this(intake, drive, 1000);
    }

    public IntakeCycleDrive(IntakeSubsystem intake, DriveSubsystem drive, long duration){
        addCommands(
            new IntakeInDrive(intake, drive, duration),
            new IntakeArmUp(intake)
        );
    }
}
