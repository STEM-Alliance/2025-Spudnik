// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.management.OperatingSystemMXBean;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    // private final CommandXboxController debugXbox = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;
    private PhotonCamera m_photonCamera = new PhotonCamera("Cam3WFOV");

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    // private final PhotonVisionSubsystem m_photonVisionSubsystem3 = new PhotonVisionSubsystem(swerveDriveSubsystem);

    // private final LEDSubsystem ledSubsystem = new LEDSubsystem(new AddressableLED(0));

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID(), m_photonCamera);

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings

        configureBindings();

        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
    }

    // Command shootAction =
    // Command alignAction = ; // Self-deadlines
    // Command spoolAction =
    // Command intakeAction = ;

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     * joysticks}. 
     */
    private void configureBindings() {
<<<<<<< HEAD
    //    driverXbox.y().whileTrue(new AimbotCommand(swerveDriveSubsystem, m_photonCamera));
       driverXbox.y().onTrue(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Aimbot);
       })).onFalse(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
       }));
       driverXbox.b().onTrue(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Home);
       })).onFalse(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
       }));
       driverXbox.leftBumper().onTrue(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.AimLeft);
       })).onFalse(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
       }));
       driverXbox.rightBumper().onTrue(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.AimRight);
       })).onFalse(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
       }));
=======
       operatorXbox.a().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV1));
       operatorXbox.b().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV2));
       operatorXbox.x().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV3));
       operatorXbox.y().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV4));
       elevatorSubsystem.setDefaultCommand(new ManualElevator(() -> operatorXbox.getLeftY(), elevatorSubsystem));
>>>>>>> Elevator_v1
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveDriveSubsystem;
    }

    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }

    public CommandXboxController getOperatorXbox() {
        return operatorXbox;
        return operatorXbox; // operats x box
    }
}
