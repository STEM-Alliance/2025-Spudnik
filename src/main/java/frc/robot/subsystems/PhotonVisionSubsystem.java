package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera m_photonCamera = new PhotonCamera("Cam3WFOV");
    private SwerveSubsystem swerveSubsystem;
    private double targetOffset = 0;
    

    public PhotonVisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }
    
    @Override
    public void periodic() {
        var result = m_photonCamera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("Target Yaw", target.getYaw());
            swerveSubsystem.setChassisSpeedsAUTO(
                new ChassisSpeeds(0,0,target.getYaw() * 0.)
            );
        }
    }

    

}
