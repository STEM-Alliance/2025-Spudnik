// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private SparkMax climbMotor;
  private double goal = 0;

  public ClimberSubsystem() {
    climbMotor = new SparkMax(ClimberConstants.climbMotorPort, MotorType.kBrushless);
    climbMotor.getEncoder().setPosition(0);
  }

  public void up() {
    goal = ClimberConstants.motorTop;
  }

  public void down() {
    goal = ClimberConstants.motorBottom;
  }

  @Override
  public void periodic() {
    double position = climbMotor.getEncoder().getPosition();
    double speed = ClimberConstants.pidController.calculate(position,goal);
    if (ClimberConstants.pidController.atSetpoint() || goal == 0) {
      climbMotor.set(0);
    } else {
      climbMotor.set(speed);
    }
  }
}
