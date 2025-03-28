package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Joint2Subsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;

    public Joint2Subsystem() {
        motor = new SparkMax(Constants.OperatorConstants.Joint2MotorID, MotorType.kBrushless);
        pidController = new PIDController(Constants.PIDConstants.Joint2P, Constants.PIDConstants.Joint2I, Constants.PIDConstants.Joint2D);
        encoder = motor.getEncoder();
    }

    public void setSetpoint(Rotation2d setpoint) {
        double angle = setpoint.getDegrees();
        double output = pidController.calculate(getCurrentAngle(), angle);
        motor.set(output);
    }

    private double getCurrentAngle() {
        // Read the current angle from the encoder
        double encoderPosition = encoder.getPosition();
        // Convert encoder position to degrees (assuming 1 rotation = 360 degrees)
        double currentAngle = encoderPosition * 360.0;
        return currentAngle;
    }
}