package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PWM;

public class LEDSubsystem extends SubsystemBase {

    public PWM m_leds = new PWM(0);
    boolean redTeam;
    /** Creates a new DriveSubsystem. */
    public LEDSubsystem() 
    {
      
    }

    @Override

    public void periodic() {

    }
    public void teamColor()
    {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            Robot.m_ledSubsystem.m_leds.setSpeed(0.61); // Red
        } else {
            Robot.m_ledSubsystem.m_leds.setSpeed(0.87); // Blue
        }
    }
    public void red()
    {
       m_leds.setSpeed(0.61);

    }
    
    public void blue()
    {
       m_leds.setSpeed(0.87);
    }

    
   public void AutoBlue()
   {
    // Breathing blue
    m_leds.setSpeed(-0.15);
   }

    public void purple()
    {
       m_leds.setSpeed(0.91);
    }


    public void strobe()
    {
       //White strobe
        m_leds.setSpeed(-0.05);
    }
}
