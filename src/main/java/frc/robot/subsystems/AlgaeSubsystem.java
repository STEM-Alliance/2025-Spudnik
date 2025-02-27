// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  public final SparkMax algaeIntake;
  public final SparkMax algaeManip;
  private ArmFeedforward feedforward;
  private double manipPosition;
  private DigitalInput algaeLimitSwitch;
  public AlgaeSubsystem() {
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
    feedforward = new ArmFeedforward(AlgaeConstants.kS, AlgaeConstants.kG, AlgaeConstants.kV);
    double feedforwardOutput = feedforward.calculate(algaeManip.getEncoder().getPosition(), algaeManip.getEncoder().getVelocity());
    algaeLimitSwitch = new DigitalInput(AlgaeConstants.ALGAE_LIMIT_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_INTAKE_SPEED);
    algaeManip.setVoltage(feedforward.calculate(algaeManip.getEncoder().getPosition(), algaeManip.getEncoder().getVelocity()));
    manipPosition = algaeManip.getEncoder().getPosition();
    //check for current spike(when algae )
    if (algaeLimitSwitch.get()){; //TODO test current indicator,  80 is amps
      holdAlgae();
    }
  }
  public void placeAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_PLACE_SPEED);
    algaeManip.setVoltage(0);
  }
  public void holdAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_HOLD_SPEED);
    if (manipPosition > 0) {
      algaeManip.setVoltage(feedforward.calculate(algaeManip.getEncoder().getPosition(), algaeManip.getEncoder().getVelocity()));
    }
  }
}