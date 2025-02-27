// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCoralCommand extends Command {
  private DistanceSensorSubsystem distanceSensorSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private double initialPosition = 0;
  private double goalPosition = 0;
  /** Creates a new CenterCoral. */
  public AlignCoralCommand(DistanceSensorSubsystem distanceSensorSubsystem, ElevatorSubsystem elevtorSubsystem) {
    this.distanceSensorSubsystem = distanceSensorSubsystem;
    this.elevatorSubsystem = elevtorSubsystem;
    SmartDashboard.putString("IntakeCommand", "Align");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = elevatorSubsystem.getIntakeEncoderPosition();
    goalPosition = initialPosition + CoralConstants.ALIGN_DISTANCE; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(elevatorSubsystem.getIntakeEncoderPosition());
    elevatorSubsystem.setIntake(0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getIntakeEncoderPosition() >= goalPosition;
  }
}
