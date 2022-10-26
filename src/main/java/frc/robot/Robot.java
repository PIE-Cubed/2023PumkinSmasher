package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The class that runs with the start of a match
 */
public class Robot extends TimedRobot {
  // ERROR CODES
  public static final int FAIL = -1;
  public static final int PASS =  1;
  public static final int DONE =  2;
  public static final int CONT =  3;

  // Object creation
  Drive                   drive;
  Controls                controls;
  private Hammer          hammer;

  // Variables
  //

  // Auto path
  private static final String kCenterAuto = "Center";
  private static final String kWallAuto   = "Wall";
  private static final String kHangarAuto = "Hangar";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Number of balls
  private static final int kTwoBall   = 2;
  private static final int kThreeBall = 3;
  //private static final int kFourBall  = 4;
  private int m_numBalls;
  private final SendableChooser<Integer> m_numBallsChooser = new SendableChooser<>();

  /**
   * Constructor
   */
  public Robot() {
    //Instance Creation
    drive         = new Drive();
    controls      = new Controls();
    hammer        = new Hammer();
  }

  @Override
  /**
   * robotInit()
   * Runs once when the robot is started
   */
  public void robotInit() {
    //Auto selection
    m_chooser.setDefaultOption("Wall Auto", kWallAuto);
    m_chooser.addOption("Center Auto", kCenterAuto);
    m_chooser.addOption("Hangar Auto", kHangarAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("Auto delay seconds", 0);

    //Number of Balls to grab
    m_numBallsChooser.setDefaultOption("2 ball", kTwoBall);
    m_numBallsChooser.addOption("3 ball", kThreeBall);
    //m_numBallsChooser.addOption("4 ball", kFourBall);
    SmartDashboard.putData("Number of Balls", m_numBallsChooser);
  }

  @Override
  /**
   * robotPeriodic()
   * Always runs on the robot
   */
  public void robotPeriodic() {
    // 
  }

  @Override
  /**
   * autonomousInit()
   * Runs once when Auto starts
   */
  public void autonomousInit() {
    //Choses start position
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    m_numBalls = m_numBallsChooser.getSelected();
    System.out.println("Auto path: " + m_numBalls);

    // Resets the gyro
    Drive.ahrs.zeroYaw();
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    // 
  }

  @Override
  /**
   * teleopInit()
   * Runs once at the start of TeleOp
   */
  public void teleopInit() {
    // 
  }

  @Override
  /**
   * teleopPeriodic()
   * Runs constantly during TeleOp
   */
  public void teleopPeriodic() {
    wheelControl();
    hammerControl();
  }

  @Override
  /**
   * disabledInit()
   */
  public void disabledInit() {
    // 
  }

  @Override
  /**
   * disabledPeriodic()
   * Shouldn't do anything
   */
  public void disabledPeriodic() {
    // Nothing yet...
  }

  @Override
  /**
   * testInit()
   * Runs once at the start of Test
   */
  public void testInit() {
    // 
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
    drive.testWheelAngle();
  }

  /**
   * Controls the wheels in TeleOp
   */
  private void wheelControl() {
    // Gets Joystick Values
    double driveX = controls.getDriveX();
    double driveY = controls.getDriveY();
    double driveZ = controls.getRotatePower();

    boolean encircle = controls.encircle();

    // Kills all automatic funcitons (Start on the Xbox controller)
    boolean autokill            = controls.autoKill();

    // General state changes
    if (autokill == true) {
      //
    } 

    // Manual driving
    if (encircle) {
      drive.circle(3);
    }
    else {
      drive.teleopSwerve(driveX, driveY, driveZ, false, true);
    }
  }
  
  public void hammerControl() {
    // Inputs
    double leftPower  = controls.retractHammer();
    double rightPower = controls.smashHammer() / 2;
    
    // SMASH!
    hammer.movement(leftPower, rightPower);
  }
}

//End of the Robot class