package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.JogJoint1Command;
import frc.robot.subsystems.Joint1Subsystem;
import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.cameraserver.CameraServer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix6.hardware.TalonFX;

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
private XboxController Operater;
private TalonFX clawMotor;
private SparkMax cageMotor;

private SparkMax telescopicMotor;
private RelativeEncoder encoder = telescopicMotor.getEncoder();
private SparkMax ballMotor;

private final DoubleSolenoid Endsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);
private final DoubleSolenoid clawsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6,9);
private final DoubleSolenoid Ballsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 10);
private final DoubleSolenoid Armsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 4);

private boolean BallOut = false;
private boolean Processor = false;
private boolean ground = false;
private boolean End = false;

Servo leftServo = new Servo(7);
Servo rightServo = new Servo(8);


public Robot()
  {
    instance = this;
   

    Driver = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    Operater = new XboxController(Constants.OperatorConstants.kOperatorControllerPort);



    cageMotor = new SparkMax(Constants.OperatorConstants.cageMotorID, MotorType.kBrushless);
    telescopicMotor = new SparkMax(Constants.OperatorConstants.intakeMotorID2, MotorType.kBrushless);
    ballMotor = new SparkMax(Constants.OperatorConstants.ballintakeID, MotorType.kBrushless);

    clawMotor = new TalonFX(Constants.OperatorConstants.clawMotorID);
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
     encoder.setPosition(0);
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
  double homeDegree = 0;
  double targetDegree = 100;
  // Convert degrees to encoder ticks
  double targetTicks = targetDegree * 29.8667; // 29.8667 ticks per degree
  // Get current encoder position
  double currentPosition = encoder.getPosition();
  
      //flicker/?
     if (Operater.getXButton()){
      leftServo.set(0);
      rightServo.set(1);
     } else if (Operater.getBButton()){
      leftServo.set(1);
      rightServo.set(0);
     }

     // score ball function
     // assuming wrist is tilted backwards
     // 1. extend pneumatics 
     // 2. wait
     // 3. raise arm (apply power, wait, cut power)
     // 4. release wrist 
      
     if (Operater.getXButton()) {

      Armsolenoid.set(Value.kForward);
      Timer.delay(0.5);

      telescopicMotor.set(0.1);
      Timer.delay(3);
      telescopicMotor.set(0);
      Timer.delay(.5);

      clawMotor.set(0.7);
      Timer.delay(0.5);
      clawMotor.set(0);
      Timer.delay(0.5);
  
      ballMotor.set(0.6);
      Timer.delay(0.5);
      ballMotor.set(0);
  
      clawMotor.set(-0.7);
      Timer.delay(0.5);
      clawMotor.set(0);
  
      
      telescopicMotor.set(-.1);
      Timer.delay(3);
      telescopicMotor.set(0);
      Armsolenoid.set(Value.kReverse);
  }
  
      
  
      // Wrist motor control

    // Set the motor speed based on trigger values
    if (Operater.getAButtonPressed()){
      cageMotor.set(-.37); 
    } else if (Operater.getRightTriggerAxis() > 0.1) {
        // Move motor forward
        cageMotor.set(-.35); // Scale speed down to 50%
    } else if (Operater.getLeftTriggerAxis() > 0.1) {
        // Move motor backward
        cageMotor.set(.8); // Scale speed down to 50%
    } else if (Operater.getLeftTriggerAxis() < 0.1  && Operater.getRightTriggerAxis() < 0.1){
        // Stop motor
        cageMotor.set(0);
    }

    //on and off for hang cylinder

    if(Driver.getPOV() == 0)
    { Endsolenoid.set(Value.kForward);
    } else if (Driver.getPOV() == 180) 
    { Endsolenoid.set(Value.kReverse);
    }

    /* 
    if(Operater.getYButtonPressed())
    { clawsolenoid.set(Value.kForward);
    } else if (Operater.getYButtonReleased())
    { clawsolenoid.set(Value.kReverse);
    }
    */

    if(Operater.getPOV() == 0)
    { Armsolenoid.set(Value.kForward);
    } else if (Operater.getPOV() == 180)
    {Armsolenoid.set(Value.kReverse);
    }
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
