package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem.AlgaePosition;

// We know that this is going away, we can use it for now
@SuppressWarnings("removal")
public class AlgaeSubsystem extends ProfiledPIDSubsystem {

    private final SparkMax algaeArmMotor = new SparkMax(AlgaeConstants.ALGAE_ARM_PORT, MotorType.kBrushless);
    private final SparkMax algaeIntakeMotor = new SparkMax(AlgaeConstants.ALGAE_INTAKE_PORT, MotorType.kBrushless);

    private final SparkAbsoluteEncoder armEmcoder;
    private AlgaePosition algaePostion = AlgaePosition.STOWED;

    private Mechanism2d mechanism = new Mechanism2d(3, 3);
    private MechanismRoot2d root;
    private MechanismLigament2d algaeMechanismLigament = new MechanismLigament2d("Algae Arm",
        Units.inchesToMeters(22), 0.0);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 25, 3.25,
            Units.inchesToMeters(22), Units.degreesToRadians(100), Units.degreesToRadians(-95), true, 0);

    /* Represents the value of the arm in degrees at a certain state */
    public enum AlgaePosition {
        STOWED(-90),
        PROCESSOR(-80),
        L2(0),
        L3(20);

        private double position;

        private AlgaePosition(double position) {
            this.position = position;
        }
    }

    public AlgaeSubsystem() {
        super(AlgaeConstants.ALGAE_ARM_CONTROLLER);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(false);

        algaeIntakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(false);

        algaeArmMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        armEmcoder = algaeArmMotor.getAbsoluteEncoder();

        root = this.mechanism.getRoot("Arm", 1.5, Units.inchesToMeters(12));
        algaeMechanismLigament.setColor(new Color8Bit("#46FF09"));

        root.append(algaeMechanismLigament);

        disable();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Arm Angle", Units.rotationsToRadians(armEmcoder.getPosition())); 
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double calculatedOutput = output
                + AlgaeConstants.ALGAE_FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);

        algaeArmMotor.setVoltage(calculatedOutput);
    }

    @Override
    protected double getMeasurement() {
        return Robot.isReal() ? Units.rotationsToRadians(armEmcoder.getPosition()) : armSim.getAngleRads();
    }

    public void setPostion(AlgaePosition position) {
        this.algaePostion = position;
        setGoal(Units.degreesToRadians(this.algaePostion.position));

        SmartDashboard.putString("Algae Position", position.name());
    }

    public void disable() {
        super.disable();
    }

    public void enable() {
        super.enable();
    }

    public void intakeAlgae() {
        algaeIntakeMotor.setVoltage(1.5);
    }

    public void extakeAlgae() {
        algaeIntakeMotor.set(-0.2);
    }

    public void setIntakeMotor(double speed) {
        algaeIntakeMotor.set(speed);
    }

    public boolean hasAlgae() {
        // TODO: No idea on the current needed for this, 1.5 is just a guess
        return algaeIntakeMotor.getOutputCurrent() > 1.5;
    }

    public SequentialCommandGroup algaeMoveCommand(AlgaePosition position) {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            this.setPostion(position);
        }), new ConditionalCommand(new InstantCommand(() -> this.intakeAlgae()), new InstantCommand(),
                new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return hasAlgae();
                    };
                }));
    }
}
