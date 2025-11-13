package org.frc3512.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public class WristIOInputs {

    public double motorVoltage = 0.0; // Volts
    public double motorCurrent = 0.0; // Amps
    public double motorPosition = 0.0; // Degrees
    public double motorVelocity = 0.0; // Degrees per Second
    public double motorAcceleration = 0.0; // Degrees per Second Squared
    public double positionSetpoint = 0.0; // Degrees
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void changeSetpoint(WristStates newSetpoint) {}

  public default String getState() {
    return WristStates.STOW.state;
  }
}
