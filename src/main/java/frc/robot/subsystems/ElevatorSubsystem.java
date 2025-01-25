// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax elevatorLeader;
  private final SparkMax elevatorFollower;
  private final DigitalInput elevatorLimitSwitch;
  public ElevatorSubsystem() {
    elevatorLeader = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_PORT, MotorType.kBrushless);
    elevatorFollower = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_PORT, MotorType.kBrushless);
    SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
    elevatorLeaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    elevatorLeaderConfig.inverted(false);
    SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    elevatorFollowerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    elevatorFollowerConfig.inverted(false);
    elevatorFollowerConfig.follow(elevatorLeader, true);
    elevatorLimitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void elevatorMove(double Speed) {
    if ((elevatorLimitSwitch.get() && Speed > 0)
     || (elevatorLeader.getEncoder().getPosition() < ElevatorConstants.ELEVATOR_TOP_LIMIT)
     || (elevatorLeader.getEncoder().getPosition() > ElevatorConstants.ELEVATOR_BOTTOM_LIMIT)){
      Speed = 0; //stops elevator movement to stop it from breaking the robot
    }
    if (elevatorLimitSwitch.get()){
      zeroElevator(); //resets elevator encoder pos to 0
    }
    elevatorLeader.set(Speed);

  }
  public void zeroElevator(){
    elevatorLeader.getEncoder().setPosition(0);

  }
}
