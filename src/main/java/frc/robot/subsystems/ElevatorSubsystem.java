// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.fasterxml.jackson.databind.deser.std.TokenBufferDeserializer;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax elevatorLeader;
  private final SparkMax elevatorFollower;
  private final SparkMax coralLeader;
  private final SparkMax coralFollower;
  private final DigitalInput elevatorLimitSwitch;
  private final DigitalInput intakeLimitSwitch;
  private final PIDController pidController;
  private Boolean inTolerance = false;
  private boolean wasInView = false;
  private ElevatorFeedforward feedforward;
  private double tunekP = 0.1;
  private DistanceSensorSubsystem distanceSensorSubsystem;

  public enum ElevatorState {
    Manual,
    L1, 
    L2, 
    L3, 
    L4,
    Park,
    Intake

  }

  private ElevatorState elevatorState = ElevatorState.Park;

  private void setElasticVisual(double height) {
    SmartDashboard.putBoolean("Intake", height == -1);
    SmartDashboard.putBoolean("L1", height > 0);
    SmartDashboard.putBoolean("L2", height > 1);
    SmartDashboard.putBoolean("L3", height > 2);
    SmartDashboard.putBoolean("L4", height > 3);
  }

  public void setElevatorState(ElevatorState elevatorState) {
    this.elevatorState = elevatorState;
    
  }

  public void resetElevatorState() {
    setElevatorState(elevatorState.Park);
  }

  public ElevatorSubsystem(DistanceSensorSubsystem distanceSensorSubsystem) {
    
    setElasticVisual(0);

    this.distanceSensorSubsystem = distanceSensorSubsystem;
    elevatorLeader = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_PORT, MotorType.kBrushless);
    elevatorFollower = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_PORT, MotorType.kBrushless);
    SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
    elevatorLeaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    elevatorLeaderConfig.inverted(false);

    elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    elevatorFollowerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    elevatorFollowerConfig.inverted(false);
    elevatorFollowerConfig.follow(elevatorLeader, true);
    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorLimitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH);
    intakeLimitSwitch = new DigitalInput(ElevatorConstants.INTAKE_LIMIT_SWITCH);

    coralLeader = new SparkMax(ElevatorConstants.CORAL_LEADER_PORT, MotorType.kBrushless);
    
    coralFollower = new SparkMax(ElevatorConstants.CORAL_FOLLOWER_PORT, MotorType.kBrushless);
    SparkMaxConfig coralLeaderConfig = new SparkMaxConfig();
    coralLeaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    coralLeaderConfig.inverted(false);
    coralLeader.configure(coralLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    SparkMaxConfig coralFollowerConfig = new SparkMaxConfig();
    coralFollowerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    coralFollowerConfig.inverted(false);
    coralFollowerConfig.follow(coralLeader, false);
    //TODO: DO NOT PUT BACK IN UNTIL FIXED
    coralFollower.configure(coralFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = new PIDController(tunekP, ElevatorConstants.kI, ElevatorConstants.kD);
    pidController.setTolerance(ElevatorConstants.PID_TOLERANCE);
    feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    SmartDashboard.putNumber("TuneKp", tunekP);
    SmartDashboard.putNumber("Elevator Goal", 0.45);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidController.setP(SmartDashboard.getNumber("TuneKp", 0));
    SmartDashboard.putNumber("Elevator Position", elevatorLeader.getEncoder().getPosition());
    SmartDashboard.putString("Elevator State", elevatorState.name());

    if (distanceSensorSubsystem.hasCoral() != wasInView) {
      Notification notification = new Notification();
      notification.setTitle("Coral");
      notification.setLevel(NotificationLevel.INFO);
      notification.setDescription(distanceSensorSubsystem.hasCoral() ? "Coral Found" : "Coral Lost");
      Elastic.sendNotification(notification);
      wasInView = distanceSensorSubsystem.hasCoral();
    }

    if (distanceSensorSubsystem.hasCoral()) {
      switch (elevatorState) {
        case Park:
          setElasticVisual(0);
          setPosition(ElevatorConstants.ELEVATOR_PARK_HEIGHT);
          break;
        case L1:
          setElasticVisual(1);
          setPosition(ElevatorConstants.LV1);
          break;
        case L2:
          setElasticVisual(2);
          setPosition(ElevatorConstants.LV2);
          break;
        case L3:
          setElasticVisual(3);
          setPosition(ElevatorConstants.LV3);
          break;
        case L4:
          setElasticVisual(4);
          setPosition(ElevatorConstants.LV4);
          break;
        case Intake:
          setElasticVisual(-1);
          setPosition(ElevatorConstants.Intake);
          break;
        case Manual:
          break;
      }
    } else {
      setElasticVisual(-1);
      setPosition(SmartDashboard.getNumber("Elevator Goal", 0.45));
    }
   
  }

  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  public void elevatorMove(double Speed) {
    if ((elevatorLimitSwitch.get() && Speed < 0)
     || (elevatorLeader.getEncoder().getPosition() > ElevatorConstants.ELEVATOR_TOP_LIMIT)
     || (elevatorLeader.getEncoder().getPosition() < ElevatorConstants.ELEVATOR_BOTTOM_LIMIT)){
      Speed = 0; //stops robot from killing itself
    }
    if (elevatorLimitSwitch.get()){
      zeroElevator(); //resets elevator encoder pos to 0
    }
    
    if (Speed > 1){
      Speed = 1;
    }else if(Speed < -1){
      Speed = -1;
    }

    elevatorLeader.set(Speed);

  }
  public void zeroElevator(){
    elevatorLeader.getEncoder().setPosition(0);

  }

  public void setPosition(double goalPosition){
    inTolerance = pidController.atSetpoint();
    pidController.setSetpoint(-goalPosition);

    double pidOutput = pidController.calculate(elevatorLeader.getEncoder().getPosition(), goalPosition);
    double feedforwardOutput = feedforward.calculate(elevatorLeader.getEncoder().getPosition(), elevatorLeader.getEncoder().getVelocity());
    double speed = pidOutput + feedforwardOutput;
    if (speed > 1){
      speed = 1;
    }else if(speed < -1){
        speed = -1;
      }
    if(elevatorLimitSwitch.get() && speed < 0){
      speed = 0;
      zeroElevator();
    }
    elevatorLeader.set(speed*ElevatorConstants.ELEVATOR_SPEED_MODIFIER);
    // System.out.println(elevatorLeader.get());
  }

  // public void setPosition(ElevatorHeights elevatorHeight) {

  //   inTolerance = pidController.atSetpoint();
  //   pidController.setSetpoint(-elevatorHeight.height);

  //   double pidOutput = pidController.calculate(elevatorLeader.getEncoder().getPosition(), elevatorHeight.height);
  //   double feedforwardOutput = feedforward.calculate(elevatorLeader.getEncoder().getPosition(), elevatorLeader.getEncoder().getVelocity());
  //   double speed = pidOutput + feedforwardOutput;
  //   if (speed > 1){
  //     speed = 1;
  //   }else if(speed < -1){
  //       speed = -1;
  //     }
  //   if(elevatorLimitSwitch.get() && speed < 0){
  //     speed = 0;
  //   }
  //   elevatorLeader.set(speed*ElevatorConstants.ELEVATOR_SPEED_MODIFIER);
  // }

  public void StopElevator(){
    elevatorLeader.stopMotor();
  }
  public boolean getInTolerance(){
    return inTolerance;
  }
  public double getElevatorPosition(){
    return elevatorLeader.getEncoder().getPosition();
  }
  
  public void setIntake(double speed) {
    coralLeader.set(speed);
  }

  public double getIntakeEncoderPosition() {
    return coralLeader.getEncoder().getPosition();
  }

  public SparkMax getCoralFollower() {
    return coralFollower;
  }
}