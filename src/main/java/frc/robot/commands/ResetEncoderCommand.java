package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Joint1Subsystem;

public class ResetEncoderCommand extends Command {
    private final Joint1Subsystem joint1Subsystem;

    public ResetEncoderCommand(Joint1Subsystem joint1Subsystem) {
        this.joint1Subsystem = joint1Subsystem;
        addRequirements(joint1Subsystem);
    }

    @Override
    public void initialize() {
        joint1Subsystem.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes immediately after resetting the encoder
    }

}
