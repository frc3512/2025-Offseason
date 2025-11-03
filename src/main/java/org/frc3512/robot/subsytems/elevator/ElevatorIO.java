package org.frc3512.robot.subsytems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double motorVoltage = 0.0; // Volts
    public double motorCurrent = 0.0; // Amps
    public double motorPosition = 0.0; // Inches
    public double motorVelocity = 0.0; // Inches per Second
    public double motorAcceleration = 0.0; // Inches per Second Squared
    public double positionSetpoint = 0.0; // Inches
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void changeSetpoint(ElevatorStates newSetpoint) {}


}
