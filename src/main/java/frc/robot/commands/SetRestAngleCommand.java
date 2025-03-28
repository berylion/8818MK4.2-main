package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;  
import frc.robot.subsystems.Joint1Subsystem;

public class SetRestAngleCommand extends Command{
    private final Joint1Subsystem joint1Subsystem;
    private final double restAngle;

    public SetRestAngleCommand(Joint1Subsystem joint1Subsystem, double restAngle) {
        this.joint1Subsystem = joint1Subsystem;
        this.restAngle = restAngle;
        addRequirements(joint1Subsystem);
    }
    

    @Override
    public void initialize() {
        joint1Subsystem.setRestAngle(restAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes immediately after setting the rest angle
    }

}
