// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AlignCoralCommand;
import frc.robot.commands.CenterCoralCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.ElasticSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

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
    private PhotonCamera m_photonCamera = new PhotonCamera("driveCamera");
    private final ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    private final DistanceSensorSubsystem distanceSensorSubsystem = new DistanceSensorSubsystem(0);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(distanceSensorSubsystem);
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    //need can id of sensor to declare next line
    //private final TofDistanceSubsystem tofDistanceSubsystem = new TofDistanceSubsystem();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID(), m_photonCamera);

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        // Configure the trigger bindings

        // TODO: Change Speed To Correct Value
        NamedCommands.registerCommand("PlaceCoral", new InstantCommand(() -> elevatorSubsystem.setIntake(0.5)));

        NamedCommands.registerCommand("L0", new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorState.Park)));
        NamedCommands.registerCommand("L1", new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorState.L1)));
        NamedCommands.registerCommand("L2", new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorState.L2)));
        NamedCommands.registerCommand("L3", new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorState.L3)));
        NamedCommands.registerCommand("L4", new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorState.L4)));
        
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
     */
    private void configureBindings() {
        operatorXbox.leftBumper()
                .onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.Intake);}))
                .onTrue(new SequentialCommandGroup(
                            new IntakeCommand(distanceSensorSubsystem, elevatorSubsystem, 0.18),
                            // new InstantCommand(() -> {

                            //     operatorXbox.setRumble(RumbleType.kBothRumble, 1);
                            // }),
                            new CenterCoralCommand(distanceSensorSubsystem, elevatorSubsystem),
                            new AlignCoralCommand(distanceSensorSubsystem, elevatorSubsystem),
                            new InstantCommand(() -> {
                                elevatorSubsystem.setElevatorState(ElevatorState.Park);
                                // operatorXbox.setRumble(RumbleType.kBothRumble, 0);
                            })
                        ));

        operatorXbox.rightBumper().onTrue(new InstantCommand(() -> {
            elevatorSubsystem.setIntake(0.3);
        })).onFalse(new InstantCommand(() -> {
            elevatorSubsystem.setIntake(0);
        }));
        
        operatorXbox.leftTrigger().and(new BooleanSupplier() {

			@Override
			public boolean getAsBoolean() {
				return operatorXbox.getLeftTriggerAxis() > 0.2;
			}
            
        }).onTrue(new InstantCommand(() -> {algaeSubsystem.placeAlgae();
        }));

        operatorXbox.povUp().onTrue(new InstantCommand(() -> {
            climberSubsystem.up();
        }));

        operatorXbox.povDown().onTrue(new InstantCommand(() -> {
            climberSubsystem.down();
        }));


        // operatorXbox.a().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV1));
        // operatorXbox.b().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV2));
        // operatorXbox.x().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV3));
        // operatorXbox.y().whileTrue(new PositionElevator(elevatorSubsystem, ElevatorConstants.LV4));
        

        operatorXbox.a().onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.L1);})).onFalse(new InstantCommand(() -> {elevatorSubsystem.resetElevatorState();}));
        operatorXbox.b().onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.L2);})).onFalse(new InstantCommand(() -> {elevatorSubsystem.resetElevatorState();}));
        operatorXbox.x().onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.L3);})).onFalse(new InstantCommand(() -> {elevatorSubsystem.resetElevatorState();}));
        operatorXbox.y().onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.L4);})).onFalse(new InstantCommand(() -> {elevatorSubsystem.resetElevatorState();}));
        operatorXbox.button(7).onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.Intake);})).onFalse(new InstantCommand(() -> {elevatorSubsystem.resetElevatorState();}));

        //operatorXbox.button(8).onFalse(new PositionElevator(elevatorSubsystem, ElevatorConstants.ELEVATOR_PARK_HEIGHT));

        //elevatorSubsystem.setDefaultCommand(new ManualElevator(() -> operatorXbox.getLeftY(), elevatorSubsystem));
        //operatorXbox.leftStick().onTrue(new InstantCommand(() -> {elevatorSubsystem.setElevatorState(ElevatorState.L4);})).onFalse(new InstantCommand(() -> {elevatorSubsystem.resetElevatorState();}));

    //    driverXbox.y().whileTrue(new AimbotCommand(swerveDriveSubsystem, m_photonCamera));
       driverXbox.y().onTrue(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Aimbot);
       })).onFalse(new InstantCommand(() -> {
        swerveDriveSubsystem.setRotationStyle(RotationStyle.Driver);
       }));
       driverXbox.b().onTrue(new InstantCommand(() -> {
        Notification notification = new Notification();
        notification.setLevel(NotificationLevel.INFO);
        notification.setTitle("State");
        notification.setDescription("Changed to \"Homing\"");
        Elastic.sendNotification(notification);
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
        return operatorXbox; // operats x box
    }
}
