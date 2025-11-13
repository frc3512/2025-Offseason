package org.frc3512.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public class ArmIOInputs {

    public double motorVoltage = 0.0; // Volts
    public double motorCurrent = 0.0; // Amps
    public double motorPosition = 0.0; // Degrees
    public double motorVelocity = 0.0; // Degrees per Second
    public double motorAcceleration = 0.0; // Degrees per Second Squared
    public double positionSetpoint = 0.0; // Degrees
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void changeSetpoint(ArmStates newSetpoint) {}

  public default String getState() {
    return ArmStates.STOW.state;
  }
}
