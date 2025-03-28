// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  





  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(12.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;

  public static final int Joint1MotorID = 17; // CAN ID for Joint 1 motor   this one
  public static final int Joint2MotorID = 24; 
  public static final int cageMotorID = 13; // CAN ID for cage motor   
  public static final int intakeMotorID2 = 15; // CAN ID for intake motor 2   
  public static final int ballintakeID = 16;
  public static final int clawMotorID = 19; // CAN ID for claw motor

  }
  public static class PIDConstants {
    public static final double Joint1P = 0.01;
    public static final double Joint1I = 0.0;
    public static final double Joint1D = 0.0;

    public static final double Joint2P = 0.1;
    public static final double Joint2I = 0.0;
    public static final double Joint2D = 0.0;
}
public static class FeedforwardConstants {
  public static final double kS = 0.2; // Static gain
  public static final double kG = 10.54; // Gravity gain
  public static final double kV = 0.31; // Velocity gain
  public static final double kA = 0.93; // Acceleration gain
}

}
