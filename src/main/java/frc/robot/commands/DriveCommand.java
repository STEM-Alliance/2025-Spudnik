package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;
    private PhotonCamera m_photonCamera;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    // Auto rotation pid/rate limiter
    private PIDController rotationController = new PIDController(6, 0.0, 0.5);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    private enum DriveState {
        Free,
        Locked,
    };

    private DriveState state = DriveState.Free;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox, PhotonCamera m_photonCamera) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;
        this.m_photonCamera = m_photonCamera;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(swerveSubsystem);
    }

    double clamp(double v, double mi, double ma) {
        return (v < mi) ? mi : (v > ma ? ma : v);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        Translation2d xyRaw = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
        Translation2d xySpeed = DeadBand(xyRaw, 0.15 / 2.f);
        double zSpeed = -DeadBand(xbox.getRightX(), 0.15);
        double xSpeed = -xySpeed.getX(); // xbox.getLeftX();
        double ySpeed = xySpeed.getY(); // xbox.getLeftY();

        // System.out.println("DRIVE!!");

        // double mag_xy = Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed);

        // xSpeed = mag_xy > 0.15 ? xSpeed : 0.0;
        // ySpeed = mag_xy > 0.15 ? ySpeed : 0.0;
        // zSpeed = Math.abs(zSpeed) > 0.15 ? zSpeed : 0.0;

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        // double dmult = dsratelimiter.calculate(xbox.getRightBumper() ? 1.0 :
        // SLOWMODE_MULT);
        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        if (xbox.getXButton()) {

            swerveSubsystem.zeroHeading();
            Translation2d pospose = swerveSubsystem.getPose().getTranslation();
            swerveSubsystem.odometry.resetPosition(swerveSubsystem.getRotation2d(),
                    swerveSubsystem.getModulePositions(),
                    new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI)));
            // swerveSubsystem.resetOdometry(new Pose2d(1.38, 5.55, new Rotation2d()));
            // swerveSubsystem.zeroHeading();
        }
        var result = m_photonCamera.getLatestResult();

        ChassisSpeeds speeds = new ChassisSpeeds();

        switch (swerveSubsystem.getRotationStyle()) {
            case Aimbot:

                if (result.hasTargets()) {
                    double yaw = getResultYaw(result);
                    SmartDashboard.putNumber("Target Yaw", yaw);
                    double deltaYaw = AimbotConstants.pidController.calculate(yaw, 0);
                    zSpeed = -deltaYaw;
                }

                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, zSpeed,
                        new Rotation2d(
                                -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET)
                                        .getRadians()));
                break;
            case Home:

                if (result.hasTargets()) {
                    double yaw = getResultYaw(result);                    
                    SmartDashboard.putNumber("Target Yaw", yaw);

                    double deltaYaw = AimbotConstants.pidController.calculate(yaw, 0);
                    zSpeed = -deltaYaw;

                    xSpeed = xbox.getLeftY() * Math.sin(Math.toRadians(-deltaYaw));
                    ySpeed = xbox.getLeftY() * Math.cos(Math.toRadians(-deltaYaw));
                   
                    SmartDashboard.putNumber("Xspeed", xSpeed);
                    SmartDashboard.putNumber("Yspeed", ySpeed);
                }
                speeds = new ChassisSpeeds(-xSpeed, ySpeed, -zSpeed);

                break;
            case AimLeft:

                if (result.hasTargets()) {
                    double distance = result.getBestTarget().getBestCameraToTarget().getMeasureX().baseUnitMagnitude();
                    double xOffset = Units.inchesToMeters(6.5);
                    double angleOffset = Math.atan(xOffset/distance);

                    double yaw = getResultYaw(result);
                    SmartDashboard.putNumber("Target Yaw", yaw);
                    double deltaYaw = AimbotConstants.pidController.calculate(yaw - Units.radiansToDegrees(angleOffset), 0);
                    zSpeed = -deltaYaw;

                    xSpeed = xbox.getLeftY() * Math.sin(Math.toRadians(-deltaYaw) + angleOffset);
                    ySpeed = xbox.getLeftY() * Math.cos(Math.toRadians(-deltaYaw) + angleOffset);
                
                    SmartDashboard.putNumber("Xspeed", xSpeed);
                    SmartDashboard.putNumber("Yspeed", ySpeed);
                } else {
                    xSpeed = 0;
                    ySpeed = xbox.getLeftY();
                    zSpeed = 0;
                }

                speeds = new ChassisSpeeds(-xSpeed, ySpeed, -zSpeed);

                break;
            case AimRight:

                if (result.hasTargets()) {
                    double distance = result.getBestTarget().getBestCameraToTarget().getMeasureX().baseUnitMagnitude();
                    double xOffset = Units.inchesToMeters(6.5);
                    double angleOffset = Math.atan(xOffset/distance);

                    double yaw = getResultYaw(result);
                    SmartDashboard.putNumber("Target Yaw", yaw);
                    double deltaYaw = AimbotConstants.pidController.calculate(yaw + Units.radiansToDegrees(angleOffset), 0);
                    zSpeed = -deltaYaw;

                    xSpeed = xbox.getLeftY() * Math.sin(Math.toRadians(-deltaYaw) - angleOffset);
                    ySpeed = xbox.getLeftY() * Math.cos(Math.toRadians(-deltaYaw) - angleOffset);
                
                    SmartDashboard.putNumber("Xspeed", xSpeed);
                    SmartDashboard.putNumber("Yspeed", ySpeed);
                } else {
                    xSpeed = 0;
                    ySpeed = xbox.getLeftY();
                    zSpeed = 0;
                }
                speeds = new ChassisSpeeds(-xSpeed, ySpeed, -zSpeed);

                break;
            case Driver:
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, zSpeed,
                        new Rotation2d(
                                -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET)
                                        .getRadians()));
                break;

        }

        // State transition logic
        switch (state) {
            case Free:
                // state = xbox.getRightBumper() ? DriveState.Locked : DriveState.Free;
                break;
            case Locked:
                // state = ((xyRaw.getNorm() > 0.15) && !xbox.getBButton()) ? DriveState.Free :
                // DriveState.Locked;
                break;
        }

        // Drive execution logic
        switch (state) {
            case Free:
                SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
                swerveSubsystem.setModules(calculatedModuleStates);
                break;
            case Locked:
                swerveSubsystem.setXstance();
                break;
        }
    }

 
    
    private double getResultYaw(PhotonPipelineResult result) {
        return result.getBestTarget().getYaw();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}
