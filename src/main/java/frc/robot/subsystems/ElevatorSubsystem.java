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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
  private final PIDController pidController;
  private Boolean inTolerance = false;
  private ElevatorFeedforward feedforward;

  public enum ElevatorHeights {

    // TODO: FIND REAL VALUES
    L0(0.1),
    L1(Units.inchesToMeters(18)),
    L2(Units.inchesToMeters(31.875)),
    L3(Units.inchesToMeters(47.625)),
    L4(Units.inchesToMeters(72));

    private double height;

    private ElevatorHeights(double height) {
      this.height = height;
    }
  }

  private ElevatorHeights elevatorHeights = ElevatorHeights.L0;

  public ElevatorHeights getElevatorHeights() {
    return elevatorHeights;
  }

  public void setElevatorHeights(ElevatorHeights elevatorHeights) {
    this.elevatorHeights = elevatorHeights;
    setPosition(elevatorHeights);
    //VISUALS
    SmartDashboard.putString("Elevator Position", elevatorHeights.name());
    SmartDashboard.putBoolean("L1", elevatorHeights.ordinal() > 0);
    SmartDashboard.putBoolean("L2", elevatorHeights.ordinal() > 1);
    SmartDashboard.putBoolean("L3", elevatorHeights.ordinal() > 2);
    SmartDashboard.putBoolean("L4", elevatorHeights.ordinal() > 3);
   
  }

  @Deprecated(forRemoval = true)
  public void up() {
    setElevatorHeights(ElevatorHeights.values()[Math.min(ElevatorHeights.values().length - 1, elevatorHeights.ordinal() + 1)]);;
  }

  @Deprecated(forRemoval = true)
  public void down() {
    setElevatorHeights(ElevatorHeights.values()[Math.max(0, elevatorHeights.ordinal() - 1)]);
  }

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

    coralLeader = new SparkMax(ElevatorConstants.CORAL_LEADER_PORT, MotorType.kBrushless);
    coralFollower = new SparkMax(ElevatorConstants.CORAL_FOLLOWER_PORT, MotorType.kBrushless);
    SparkMaxConfig coralLeaderConfig = new SparkMaxConfig();
    coralLeaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    coralLeaderConfig.inverted(false);
    SparkMaxConfig coralFollowerConfig = new SparkMaxConfig();
    coralFollowerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    coralFollowerConfig.inverted(false);
    coralFollowerConfig.follow(coralLeader, true);

    pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    pidController.setTolerance(ElevatorConstants.PID_TOLERANCE);
    feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

    setElevatorHeights(ElevatorHeights.L0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void elevatorMove(double Speed) {
    if ((elevatorLimitSwitch.get() && Speed > 0)
     || (elevatorLeader.getEncoder().getPosition() < ElevatorConstants.ELEVATOR_TOP_LIMIT)
     || (elevatorLeader.getEncoder().getPosition() > ElevatorConstants.ELEVATOR_BOTTOM_LIMIT)){
      Speed = 0; //stops robot from killing itself
    }
    if (elevatorLimitSwitch.get()){
      zeroElevator(); //resets elevator encoder pos to 0
    }
    elevatorLeader.set(Speed);

  }
  public void zeroElevator(){
    elevatorLeader.getEncoder().setPosition(0);

  }

  @Deprecated
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
  }

  public void setPosition(ElevatorHeights elevatorHeight) {

    inTolerance = pidController.atSetpoint();
    pidController.setSetpoint(-elevatorHeight.height);

    double pidOutput = pidController.calculate(elevatorLeader.getEncoder().getPosition(), elevatorHeight.height);
    double feedforwardOutput = feedforward.calculate(elevatorLeader.getEncoder().getPosition(), elevatorLeader.getEncoder().getVelocity());
    double speed = pidOutput + feedforwardOutput;
    if (elevatorHeights == ElevatorHeights.L0) speed = -0.5;
    if (speed > 1){
      speed = 1;
    }else if(speed < -1){
        speed = -1;
      }
    if(elevatorLimitSwitch.get() && speed < 0){
      speed = 0;
      zeroElevator();
      //TODO: This only happens at the start; make it check every so often
      if (elevatorHeight == ElevatorHeights.L0) {
        setElevatorHeights(ElevatorHeights.L1);
        Notification notification = new Notification();
        notification.setLevel(NotificationLevel.INFO);
        notification.setTitle("Elevator");
        notification.setDescription("Elevator Zeroed");
        Elastic.sendNotification(notification);
      }
    }
    elevatorLeader.set(speed*ElevatorConstants.ELEVATOR_SPEED_MODIFIER);
  }

  public void StopElevator(){
    elevatorLeader.stopMotor();
  }
  public boolean getInTolerance(){
    return inTolerance;
  }
  public double getElevatorPosition(){
    return elevatorLeader.getEncoder().getPosition();
  }
  public void intakeCoral(double speed){
    coralLeader.set(speed);
  }
  public void placeCoral(double speed){
    coralLeader.set(-speed);
  }
  public static double getPostHeight(ElevatorHeights height) {
    return height.height;
  }
}
