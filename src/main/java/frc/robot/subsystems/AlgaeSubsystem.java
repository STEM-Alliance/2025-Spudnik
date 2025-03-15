// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  public final SparkMax algaeIntake;
  public final SparkMax algaeManip;
  //private ArmFeedforward feedforward;
  private final CommandXboxController opController;
  private double manipPosition;
  private double armRotations;
  private double joystickPos;
  public AlgaeSubsystem(CommandXboxController controller) {
    opController = controller;
    algaeIntake = new SparkMax(AlgaeConstants.ALGAE_INTAKE_PORT, MotorType.kBrushless);
    SparkMaxConfig AlgaeIntakeConfig = new SparkMaxConfig();
    AlgaeIntakeConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    AlgaeIntakeConfig.inverted(false);
    algaeManip = new SparkMax(AlgaeConstants.ALGAE_MANIP_PORT, MotorType.kBrushless);
    SparkMaxConfig AlgaeManipConfig = new SparkMaxConfig();
    AlgaeManipConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    AlgaeManipConfig.inverted(false);
    manipPosition = 0;
    algaeManip.getEncoder().setPosition(0);
    //feedforward = new ArmFeedforward(AlgaeConstants.kS, AlgaeConstants.kG, AlgaeConstants.kV);
    //double feedforwardOutput = feedforward.calculate(algaeManip.getEncoder().getPosition(), algaeManip.getEncoder().getVelocity());
    //algaeLimitSwitch = new DigitalInput(AlgaeConstants.ALGAE_LIMIT_SWITCH);
    SmartDashboard.putNumber("ArmJoystick", joystickPos);
    SmartDashboard.putNumber("Arm rotations", armRotations);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmJoystick", joystickPos);
    joystickPos += opController.getRightY() * 0.01;
    armRotations = algaeManip.getEncoder().getPosition();
    //armRotations = Math.max(Math.min(-55d, armRotations), -0.01);
    SmartDashboard.putNumber("Arm rotations", armRotations);

    if(joystickPos < 0 && joystickPos > -AlgaeConstants.deadband) {
      joystickPos = 0;
    } else if (joystickPos > 0 && joystickPos < AlgaeConstants.deadband) {
      joystickPos = 0;
    } /*else if(armRotations < AlgaeConstants.TOP_LIMIT) {
      joystickPos = 0.011; //top limit is negative
    } else if(armRotations > AlgaeConstants.BOTTOM_LIMIT){
      joystickPos = -0.011;
    }*/
    
    algaeManip.set(joystickPos); 
  }
 
  public void placeAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_PLACE_SPEED);
    algaeManip.setVoltage(0);
  }
  
  public void intakeAlgaeManual(){
    algaeIntake.setVoltage(3);
  }
  public void stop(){
    algaeIntake.set(0);
  }

  public void extakeAlgae(){
    algaeIntake.set(-1);
  }

  public void setAlgaeRotations(){

  }
  public void manualArm(){
/*
    if(armRotations < AlgaeConstants.BOTTOM_LIMIT){
      armRotations = AlgaeConstants.BOTTOM_LIMIT;
      algaeManip.setVoltage(0);
    } else if(armRotations > AlgaeConstants.TOP_LIMIT){
      armRotations = AlgaeConstants.TOP_LIMIT;
      algaeManip.setVoltage(0);
    }*/
  }
}