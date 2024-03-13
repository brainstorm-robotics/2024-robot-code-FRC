package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeArmUp;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCycle extends SequentialCommandGroup{
    public IntakeCycle(IntakeSubsystem intake){
        addCommands(
            new IntakeIn(intake),
            new IntakeArmUp(intake)
        );
    }

    public IntakeCycle(IntakeSubsystem intake, long duration){
        addCommands(
            new IntakeIn(intake, duration),
            new IntakeArmUp(intake)
        );
    }
}
