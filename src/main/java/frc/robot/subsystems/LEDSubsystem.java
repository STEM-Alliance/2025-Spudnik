package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.PWM;

public class LEDSubsystem extends SubsystemBase {

    public PWM m_leds = new PWM(0);

    /** Creates a new DriveSubsystem. */
    public LEDSubsystem() 
    {
    }

    @Override

    public void periodic() {

    }

    public Command red()
    {
        return new InstantCommand(() -> m_leds.setSpeed(0.61));
    }

    public Command yellow()
    {
        return new InstantCommand(() -> m_leds.setSpeed(0.69));
    }

    public Command blue()
    {
        return new InstantCommand(() -> m_leds.setSpeed(-0.15));
    }

    public Command crazy()
    {
        return new InstantCommand(() -> m_leds.setSpeed(-0.59));
    }
}
