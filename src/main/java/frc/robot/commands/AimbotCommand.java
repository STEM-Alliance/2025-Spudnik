// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimbotConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimbotCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private PhotonCamera photonCamera;
  /** Creates a new AimbotCommand. */
  public AimbotCommand(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCamera = photonCamera;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.


  @Override
  public void execute() {
     var result = photonCamera.getLatestResult();
      if (result.hasTargets()) {
          double yaw = getResultYaw(result);
          SmartDashboard.putNumber("Target Yaw", yaw);
          double deltaYaw = AimbotConstants.pidController.calculate(yaw,0);
          swerveSubsystem.setChassisSpeedsAUTO(
              new ChassisSpeeds(0,0,-deltaYaw)
          );
          
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }

  private double getResultYaw(PhotonPipelineResult result) {
    return result.getBestTarget().getYaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var result = photonCamera.getLatestResult();
      if (result.hasTargets()) {
        double yaw = getResultYaw(result);
        if (Math.abs(yaw) < 4) {
          return true;
        }
      }
    return false;
  }
}
