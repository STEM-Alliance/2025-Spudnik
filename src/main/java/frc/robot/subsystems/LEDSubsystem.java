// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  private AddressableLED m_Led;
  private AddressableLEDBuffer m_LedBuffer;

  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance ledSpacing = Meters.of(1 / 120.0);
  private LEDPattern pattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);


  public LEDSubsystem(AddressableLED m_Led) {
    this.m_Led = m_Led;

    m_LedBuffer = new AddressableLEDBuffer(60);
    m_Led.setLength(m_LedBuffer.getLength());

    pattern.applyTo(m_LedBuffer);

    m_Led.setData(m_LedBuffer);
    m_Led.start();
  }

  public void setColor(LEDPattern pattern) {
    this.pattern = pattern;
    pattern.applyTo(m_LedBuffer);
    m_Led.setData(m_LedBuffer);
  }


  @Override
  public void periodic() {
    pattern.applyTo(m_LedBuffer);
    m_Led.setData(m_LedBuffer);

  }
}
