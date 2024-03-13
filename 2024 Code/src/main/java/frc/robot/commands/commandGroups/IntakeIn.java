package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeArmDown;
import frc.robot.commands.intake.IntakeRollerIn;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeIn extends SequentialCommandGroup{
    public IntakeIn(IntakeSubsystem intake){
        this(intake, 1000);
    }

    public IntakeIn(IntakeSubsystem intake, long duration){
        addCommands(
            new IntakeArmDown(intake),
            new IntakeRollerIn(intake, duration)
        );
    }
}
