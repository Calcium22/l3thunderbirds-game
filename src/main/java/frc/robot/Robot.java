// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//sparkmax imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
//sparkflex imports
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.TimedRobot;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  //intake motor has reverse on x pressed
  private final PWMSparkMax m_intake = new PWMSparkMax(2);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

//sparkmax motor
private final SparkMax c_canTurretMotor = new SparkMax(1, MotorType.kBrushless);
private final SparkClosedLoopController c_turretController =
    c_canTurretMotor.getClosedLoopController();
private final RelativeEncoder c_turretEncoder = c_canTurretMotor.getEncoder();

// PID constants
public static double P_gain = +0.5;
public static double Integral_gain = 0;
public static double Derivative = 0;
public static double I_Zone = 0;
public static double tMinOutput = 0.1;
public static double tMaxOutput = 1.0;
//sparkmax variables

//sparkflex
private sparkflex f_shooter;
private SparkClosedLoopController f_closedLoopController;
private static final int KdeviceId = 1;
private static final double kTargetRPM = 2000;
  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    SparkMaxConfig turretConfig = new SparkMaxConfig();

    // Configure PID controller 
    turretConfig.closedLoop.p(P_gain);
    turretConfig.closedLoop.i(Integral_gain);
    turretConfig.closedLoop.d(Derivative);
    turretConfig.closedLoop.iZone(I_Zone);
    turretConfig.closedLoop.outputRange(tMinOutput, tMaxOutput);

    c_canTurret.configure(
        Config,
        SparkMaxConfig.KResetSafeParameters,
        SparkMax.Presistmode.KPirsistParameters
    );
  
      f_shooter = new SparkFlex(KdeviceId, SparkFlex.MotorType.kBrushless);
      f_closedLoopController = f_shooter.getClosedLoopController();

      SparkFlexConfig f_config = new SparkFlexConfig();

      f_config.closedLoop.pid(0.0001, 0.0, 0.0)
        .feedFoward(0.00017)
        .outputRange(-1, 1)
        .FeedbackSensor(FeedbackSensor.kPrimaryEncoder);

        f_shooter.configure(config, Resetmode.KResetSafeParameters, PersistMode.KPirsistParameters);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }


  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public  void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY() * .5, -m_controller.getRightX() * .5);

    // Intake control
    if (m_controller.getAButtonPressed()) {
      m_intake.set(1.0); 
    } else {
      m_intake.set(0.0);
    }
    if (m_controller.getBButtonPressed()) {
      m_intake.set(-1.0); 
    } else {
      m_intake.set(0.0);
    }
    //output
    if (m_controller.getXButtonPressed()) {
      m_intake.set(1.0);
    } else {
      m_intake.set(0.0);
    }

    if (m_controller.getYButton()) {
      f_closedLoopController.setReference(kTargetRPM, ControlType.kVelocity);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
