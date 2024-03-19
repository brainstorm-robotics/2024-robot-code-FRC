package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase{
    private TalonFX leftClimber;
    private TalonFX rightClimber;

    private MotionMagicVoltage leftRequest;
    private MotionMagicVoltage rightRequest;



    public ClimberSubsystem(){
        leftClimber = new TalonFX(Climber.LEFT_ID);
        rightClimber = new TalonFX(Climber.RIGHT_ID);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        //slot 0 configs

        Slot0Configs leftSlot0 = leftConfig.Slot0;
        Slot0Configs rightSlot0 = rightConfig.Slot0;

        leftSlot0.kS = 0.25; //add S Volts because of static friction
        leftSlot0.kV = 0.12; //velocity of 1 rps requires V Volts
        leftSlot0.kA = 0.01; //accelerates 1 rps/s with A Volts
        //PID :skull:
        //don't touch these unless you know what you're doing
        leftSlot0.kP = 4.8; 
        leftSlot0.kI = 0;
        leftSlot0.kD = 0.1;

        rightSlot0.kS = 0.25; //add S Volts because of static friction
        rightSlot0.kV = 0.12; //velocity of 1 rps requires V Volts
        rightSlot0.kA = 0.01; //accelerates 1 rps/s with A Volts
        //PID :skull:
        //don't touch these unless you know what you're doing
        rightSlot0.kP = 4.8; 
        rightSlot0.kI = 0;
        rightSlot0.kD = 0.1;

        //Motion Magic
        MotionMagicConfigs leftMagicConfigs = leftConfig.MotionMagic;
        MotionMagicConfigs rightMagicConfigs = rightConfig.MotionMagic;

        leftMagicConfigs.MotionMagicCruiseVelocity = 80; //desired velocity of 80 rps
        leftMagicConfigs.MotionMagicAcceleration = 160; //desired acceleration of 160 rps/s
        leftMagicConfigs.MotionMagicJerk = 1600; //desired jerk of 1600 rps/s/s

        rightMagicConfigs.MotionMagicCruiseVelocity = 80; //desired velocity of 80 rps
        rightMagicConfigs.MotionMagicAcceleration = 160; //desired acceleration of 160 rps/s
        rightMagicConfigs.MotionMagicJerk = 1600; //desired jerk of 1600 rps/s/s

        leftClimber.getConfigurator().apply(leftConfig);
        rightClimber.getConfigurator().apply(rightConfig);

        //initialize at 0?
        leftRequest = new MotionMagicVoltage(0);
        rightRequest = new MotionMagicVoltage(0);
    }

    public void setBrakes(boolean state){
        if(state){//turn brakes on
            leftClimber.setNeutralMode(NeutralModeValue.Brake);
            rightClimber.setNeutralMode(NeutralModeValue.Brake);
            return;
        }//turn brakes off
        leftClimber.setNeutralMode(NeutralModeValue.Coast);
        rightClimber.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets the climber state to go up or down
     * @param goingUp true if going up
     * @return returns true if at the desired position
     */
    public boolean setState(boolean goingUp){
        double desired = Climber.MIN_ROTATIONS;
        if(goingUp){
            desired = Climber.MAX_ROTATIONS;
        }
        
        leftClimber.setControl(leftRequest.withPosition(desired));
        rightClimber.setControl(rightRequest.withPosition(desired));
        
        if(goingUp){
            if(leftClimber.getPosition().getValueAsDouble() >= Climber.MAX_ROTATIONS && rightClimber.getPosition().getValueAsDouble() >= Climber.MAX_ROTATIONS){
                return true;
            }
        } else{
            if(leftClimber.getPosition().getValueAsDouble() <= Climber.MIN_ROTATIONS && rightClimber.getPosition().getValueAsDouble() <= Climber.MIN_ROTATIONS){
                return true;
            }
        }
        return false;
    }
}
