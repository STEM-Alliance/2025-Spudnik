// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterCoralCommand extends Command {
  private DistanceSensorSubsystem distanceSensorSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private boolean needsCentering = true;
  /** Creates a new CenterCoral. */
  public CenterCoralCommand(DistanceSensorSubsystem distanceSensorSubsystem, ElevatorSubsystem elevtorSubsystem) {
    this.distanceSensorSubsystem = distanceSensorSubsystem;
    this.elevatorSubsystem = elevtorSubsystem;
    SmartDashboard.putString("IntakeCommand", "Center");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (distanceSensorSubsystem.get_distance() < CoralConstants.SENSOR_DISTANCE) {
      elevatorSubsystem.setIntake(-0.12);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Centering... D: " + Double.toString(distanceSensorSubsystem.get_distance()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setIntake(0);
  }

  // Returns true when the command should end.
  @Override                               
  public boolean isFinished() {
    return distanceSensorSubsystem.get_distance() > CoralConstants.SENSOR_DISTANCE;
  }
}
