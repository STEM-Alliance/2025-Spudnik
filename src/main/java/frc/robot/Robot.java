// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AlgaeSubsystemV2.AlgaeGoal;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;

  public static LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  private RobotContainer m_robotContainer;

  public static SendableChooser<String> autoChooser = new SendableChooser<>();
  //LEDstripOne m_stripOne = new LEDstripOne(9);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // ffleboard.getTab("SmartDashboard").add(autoChooser);

            DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
    m_robotContainer.getSwerveSubsystem().stopDrive();
    m_robotContainer.getSwerveSubsystem().zeroHeading();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getSwerveSubsystem().stopDrive();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_ledSubsystem.m_leds.setSpeed(-0.15);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.getSwerveSubsystem().setHeading(180);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
   // otContainer.getElevatorSubsystem().setElevatorState(ElevatorState.Reset);
   //m_ledSubsystem.blue();
    // m_ledSubsystem.m_leds.setSpeed(0);
    m_robotContainer.getAlgaeSubsystem().setAlgaeGoal(AlgaeGoal.Stowed);
    m_robotContainer.getSwerveSubsystem().setRotationStyle(RotationStyle.Driver);
    // m_robotContainer.resetShootake();
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_ledSubsystem.m_leds.setSpeed(0.61);
    } else {
      m_ledSubsystem.m_leds.setSpeed(0.87);
    }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    
  }
}