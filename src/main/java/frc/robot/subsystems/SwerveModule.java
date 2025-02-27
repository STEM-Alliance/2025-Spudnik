package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMax steerMotor;
    private final SparkMaxConfig steerConfig;

    // private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;
    private final RelativeEncoder driveMotorEncoder;

    private double driveEncSim = 0;
    private double steerEncSim = 0;

    private final com.ctre.phoenix6.hardware.CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;

    private final PIDController steerPID;

    private static int moduleNumber = 0;
    int thisModuleNumber;

    SlewRateLimiter turnratelimiter = new SlewRateLimiter(4.d);

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed) {
        // driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        System.out.println(thisModuleNumber);
        
        driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
        driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        driveConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER)
                .velocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotorEncoder = driveMotor.getEncoder();

        // driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor = new SparkMax(steerCanID, MotorType.kBrushless);
        steerConfig = new SparkMaxConfig();
        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        steerConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.STEER_ROTATION_TO_RADIANS)
                .velocityConversionFactor(SwerveModuleConstants.STEER_RADIANS_PER_MINUTE);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.motor_inv = motorReversed;
        // driveMotorEncoder = driveMotor.get();
        steerMotorEncoder = steerMotor.getEncoder();

        // Reset encoder offsets possibly set in Tuner X
        absoluteEncoder = new com.ctre.phoenix6.hardware.CANcoder(absoluteEncoderPort);
        com.ctre.phoenix6.configs.CANcoderConfiguration cfg = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        cfg.MagnetSensor = new com.ctre.phoenix6.configs.MagnetSensorConfigs();
        cfg.MagnetSensor.MagnetOffset = 0.0f;
        absoluteEncoder.getConfigurator().apply(cfg);
        // CANcoderConfigurator configurator = absoluteEncoder.getConfigurator();

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        // driveMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER);
        // driveMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);
        // TalonFXConfiguration talon_cfg = new TalonFXConfiguration();
        // driveMotor.getConfigurator().apply(cfg);

        steerPID = new PIDController(SwerveModuleConstants.MODULE_KP, 0, SwerveModuleConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        thisModuleNumber = moduleNumber;
        moduleNumber++;

        resetEncoders();
    }

    public void simulate_step() {
        driveEncSim += 0.02 * driveMotor.get() * (DriveConstants.MAX_MODULE_VELOCITY);
        steerEncSim += 0.02 * steerMotor.get() * (10.0);
    }

    public double getDrivePosition() {
        if (Robot.isSimulation())
            return driveEncSim;
        // return driveMotorEncoder.getPosition();
        // TODO: Do the conversion in the motor
        
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        // return driveMotorEncoder.getVelocity();
        return driveMotorEncoder.getVelocity();
    }

    public double getSteerPosition() {
        if (Robot.isSimulation())
            return steerEncSim;
        return getAbsoluteEncoderPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble());// * (Math.PI /
        // 180.d);
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        // driveMotorEncoder.setPosition(0);
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),
                new Rotation2d(-getSteerPosition()).rotateBy(DriveConstants.NAVX_ANGLE_OFFSET.times(-1)));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state.optimize(new Rotation2d(getSteerPosition()));
        double drive_command = state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY;
        // SmartDashboard.putNumber("Module " + Integer.toString(this.thisModuleNumber) + " Drive", drive_command);
        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        // This is stupid
        // steerPID.setP(Constants.SwerveModuleConstants.MODULE_KP *
        // Math.abs(drive_command));
        double steercmd = steerPID.calculate(getSteerPosition(), state.angle.getRadians());
        if (Robot.isSimulation()) {
            steerMotor.set(steercmd);
        } else {
            steerMotor.setVoltage(12 * steercmd);
        }
        // SmartDashboard.putNumber("Abs" + thisModuleNumber,
        // getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.1) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
