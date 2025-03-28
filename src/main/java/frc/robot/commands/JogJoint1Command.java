package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Joint1Subsystem;

public class JogJoint1Command extends Command{
    private final Joint1Subsystem joint1Subsystem;
    private final double speed;

    public JogJoint1Command(Joint1Subsystem joint1Subsystem, double speed) {
        this.joint1Subsystem = joint1Subsystem;
        this.speed = speed;
        addRequirements(joint1Subsystem);
    }

    @Override
    public void execute() {
        joint1Subsystem.jog(speed);
    }

    @Override
    public void end(boolean interrupted) {
        joint1Subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Command continues until the button is released
    } 
}
