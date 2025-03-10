// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralAlignPassthroughCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private boolean intakeInterup;
  /** Creates a new CoralAligner. */
  public CoralAlignPassthroughCommand(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
        SmartDashboard.putString("IntakeCommand", "Pass");

      }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setIntake(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_ledSubsystem.m_leds.setSpeed(0.77);
    elevatorSubsystem.setIntake(0);
    SmartDashboard.putString("IntakeCommand", "Ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getBeamBreakDI() || RobotContainer.intakeInterup;
  }
}
