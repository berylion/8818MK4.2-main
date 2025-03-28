package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Joint1Subsystem;

public class SetJoint1AngleCommand extends Command {
    private final Joint1Subsystem joint1Subsystem;
    private final Rotation2d setpoint;

    public SetJoint1AngleCommand(Joint1Subsystem joint1Subsystem, Rotation2d setpoint) {
        this.joint1Subsystem = joint1Subsystem;
        this.setpoint = setpoint;
        addRequirements(joint1Subsystem);
    }

    @Override
    public void initialize() {
        joint1Subsystem.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        // Command finishes immediately after setting the setpoint
        return true;
    }
}
