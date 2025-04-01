package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.cameraserver.CameraServer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.XboxController;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot{
private static Robot instance;
private Command m_autonomousCommand;
private RobotContainer m_robotContainer;
private Timer disabledTimer;

private XboxController Driver;
private XboxController Operator;

private SparkMax wristMotor;
private SparkMax ballMotor;
private SparkMax coralMotor;
private SparkMax Pivot1;
private SparkMax Pivot2;
// private RelativeEncoder encoder = telescopicMotor.getEncoder();

private final DoubleSolenoid Endsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);
private final DoubleSolenoid Armsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 4);

public Robot()
  {
    instance = this;
   
    Driver = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    Operator = new XboxController(Constants.OperatorConstants.kOperatorControllerPort);

    wristMotor = new SparkMax(Constants.OperatorConstants.wristMotorID, MotorType.kBrushless);
    ballMotor = new SparkMax(Constants.OperatorConstants.ballintakeID, MotorType.kBrushless);
    coralMotor = new SparkMax(Constants.OperatorConstants.coralMotorID, MotorType.kBrushless);

    Pivot1 = new SparkMax(Constants.OperatorConstants.Pivot1MotorID, MotorType.kBrushless);
    Pivot2 = new SparkMax(Constants.OperatorConstants.Pivot2MotorID, MotorType.kBrushless);
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // CameraServer.startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    Endsolenoid.set(Value.kReverse);
    Armsolenoid.set(Value.kReverse);
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(true);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // as soon as the match starts zero the lifting portion of the arm
    //  encoder.setPosition(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic(){




     /** Encoder Logic **/
     // Target position in degrees
    //  double homeDegree = 0;
    //  double targetDegree = 100;
     // Convert degrees to encoder ticks
    //  double targetTicks = targetDegree * 29.8667; // 29.8667 ticks per degree
     // Get current encoder position
     // double currentPosition = encoder.getPosition();

     // score ball function
     // assuming wrist is tilted backwards
     // 1. extend pneumatics 
     // 2. wait
     // 3. raise arm (apply power, wait, cut power)
     // 4. release wrist 
      
  //    if (Operator.getXButton()) {

  //     Armsolenoid.set(Value.kForward);
  //     Timer.delay(0.5);

  //     telescopicMotor.set(0.1);
  //     Timer.delay(3);
  //     telescopicMotor.set(0);
  //     Timer.delay(.5);

  //     wristMotor.set(0.7);
  //     Timer.delay(0.5);
  //     wristMotor.set(0);
  //     Timer.delay(0.5);
  
  //     ballMotor.set(0.6);
  //     Timer.delay(0.5);
  //     ballMotor.set(0);
  
  //     wristMotor.set(-0.7);
  //     Timer.delay(0.5);
  //     wristMotor.set(0);
      
  //     telescopicMotor.set(-.1);
  //     Timer.delay(3);
  //     telescopicMotor.set(0);
  //     Armsolenoid.set(Value.kReverse);
  // }

  // wrist control !!OPERATOR!!
  if (Operator.getPOV() == 90){
    wristMotor.set(.6);
  } else if (Operator.getPOV() == 270){
    wristMotor.set(-.6);
  } else wristMotor.stopMotor();

  // ball controls !!OPERATOR!!
   if (Operator.getYButtonPressed()){
    ballMotor.set(.8);
  } else if (Operator.getAButtonPressed()){
    ballMotor.set(-.68);
  } else if (Operator.getYButtonReleased() || Operator.getAButtonReleased()){
    ballMotor.set(0);
  }
 
  // coral subsystem control !!OPERATOR!!
  if (Operator.getXButtonPressed()){
    coralMotor.set(-.37); 
  }  else if (Operator.getStartButtonPressed()){
    coralMotor.set(.3);
  } else if (Operator.getBButtonPressed()){
    coralMotor.set(.4);
  } else if (Operator.getXButtonReleased() || Operator.getBButtonReleased() || Operator.getStartButtonReleased()){
    coralMotor.set(0);
  }

  // Arm Pivot control !!OPERATOR!!
  if (Operator.getRightBumperButtonPressed()) {
      Pivot1.set(.9);
      Pivot2.set(.9);
  } else if (Operator.getLeftBumperButtonPressed()) {
      Pivot1.set(-.9);
      Pivot2.set(-.9);
  } else if (Operator.getRightBumperButtonReleased() || Operator.getRightBumperButtonReleased()) {
      Pivot1.set(0);
      Pivot2.set(0);
  }

  //on and off for hang cylinder !!DRIVER!!
  if(Driver.getPOV() == 0){ 
      Endsolenoid.set(Value.kForward);
  } else if (Driver.getPOV() == 180) { 
      Endsolenoid.set(Value.kReverse);
  }

  //Up and down for Arm cylinders !!OPERATOR!!
  if(Operator.getPOV() == 0){ 
      Armsolenoid.set(Value.kForward);
  } else if (Operator.getPOV() == 180){
      Armsolenoid.set(Value.kReverse);
  }

  // how to do trigger controls example !!
  // (Operator.getRightTriggerAxis() > 0.1) {
  //     // Move motor forward
  //     coralMotor.set(-.35); // Scale speed down to 50%
  // } else if (Operator.getLeftTriggerAxis() > 0.1) {
  //     // Move motor backward
  //     coralMotor.set(.8); // Scale speed down to 50%
  // } else if (Operator.getLeftTriggerAxis() < 0.1  && Operator.getRightTriggerAxis() < 0.1){
  //     // Stop motor
  //     coralMotor.set(0);
  // }
}

  

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
  }
