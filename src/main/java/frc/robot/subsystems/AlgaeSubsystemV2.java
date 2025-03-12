// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeSubsystemV2 extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  

  public final SparkMax algaePivot;
  public final SparkMax algaeIntake;
  private final CommandXboxController opController;
  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.85, 0.32,0.10);//kV was 0.26
  private double armOffset = 0;
  private int holdCounter = 0;
  private boolean intaking = false;

  public enum AlgaeGoal {
    Stowed(-0.01),
    Processor(-0.05),
    L2(-0.307),
    L3(-0.453);

    public final double goal;

    private AlgaeGoal(double goal) {
      this.goal = goal;
    }
  }
  
  private AlgaeGoal algaeGoal = AlgaeGoal.Stowed;

  public AlgaeGoal getAlgaeGoal() {
    return algaeGoal;
  }

  public void setAlgaeGoal(AlgaeGoal algaeGoal) {
    if (algaeGoal != this.algaeGoal) {
      this.algaeGoal = algaeGoal;
    }
  }

  public AlgaeSubsystemV2(CommandXboxController controller) {
    opController = controller;

    algaeIntake = new SparkMax(AlgaeConstants.ALGAE_INTAKE_PORT, MotorType.kBrushless);
    algaePivot = new SparkMax(AlgaeConstants.ALGAE_MANIP_PORT, MotorType.kBrushless);

    SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
    algaeIntakeConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    algaeIntakeConfig.inverted(false);
    algaeIntake.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig algaePivotConfig = new SparkMaxConfig();
    algaePivotConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    algaePivotConfig.inverted(false);
    
    algaePivot.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    algaePivot.getEncoder().setPosition(0);
  
  }

  @Override
  public void periodic() {
    armOffset += opController.getRightY() * 0.01;

    double pidOutput = AlgaeConstants.PID_CONTROLLER.calculate(getAbsoluteEncoderPosition(), (algaeGoal.goal + armOffset));
    double feedForwardOutput = 0;// - armFeedforward.calculate((algaeGoal.goal + 0.25), 0);
    algaePivot.set((pidOutput + feedForwardOutput) / 12d); //motor inverted instead of megative

    SmartDashboard.putString("Current Alage Position", algaeGoal.name());
    SmartDashboard.putNumber("Algae Encoder Position", getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Arm Offset", armOffset);
    SmartDashboard.putBoolean("Has Algae", hasAlgae());
    SmartDashboard.putNumber("Algae Current", algaeIntake.getOutputCurrent());
    SmartDashboard.putNumber("Arm PID Output",pidOutput);
    SmartDashboard.putNumber("Arm Feedforward Output",feedForwardOutput);

    if (hasAlgae()) {
      intaking = false;
    }

    if(intaking) {
      algaeIntake.setVoltage(2);
    } else if (opController.getLeftTriggerAxis() < 0.1) {
      algaeIntake.set(0);
    }

  }

  public boolean hasAlgae() {
    return algaeIntake.getOutputCurrent() > 30;
  }

  public void intakeAlgae(){
    intaking = true;
    algaeIntake.setVoltage(3);
  }

  public void stop(){
    algaeIntake.set(0);
    intaking = false;
  }

  public void extakeAlgae(){
    algaeIntake.set(-1);
    intaking = false;
  }

   public double getAbsoluteEncoderPosition() {
       return algaePivot.getEncoder().getPosition() * (1d / 25d);
    }

    public SequentialCommandGroup moveAlgae(AlgaeGoal algaeGoal) {
      
      return new SequentialCommandGroup(new InstantCommand(() -> {
        armOffset = 0;
        setAlgaeGoal(algaeGoal);
      }), new InstantCommand(() -> {
        intakeAlgae();
      }).onlyIf(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return hasAlgae();
        }
      }));
      // }), new ConditionalCommand(new InstantCommand(() -> {
      //   intakeAlgae();
      // }), new PrintCommand("Already Has Algae!"), new BooleanSupplier() {
      //   @Override
      //   public boolean getAsBoolean() {
      //       return hasAlgae();
      //   }
      // }));
    }
 
}