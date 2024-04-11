package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveBackward;
import frc.robot.commands.intake.IntakeArmUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCycleDriveReturnShot extends SequentialCommandGroup{
    public IntakeCycleDriveReturnShot(IntakeSubsystem intake, DriveSubsystem drive, ShooterSubsystem shooter){
        this(intake, drive, shooter, 1000);
    }

    public IntakeCycleDriveReturnShot(IntakeSubsystem intake, DriveSubsystem drive, ShooterSubsystem shooter, long duration){
        addCommands(
            new IntakeInDrive(intake, drive, duration),
            new IntakeArmUp(intake),
            new DriveBackward(drive, 3500), 
            new Shoot(intake, shooter)
        );
    }
}
