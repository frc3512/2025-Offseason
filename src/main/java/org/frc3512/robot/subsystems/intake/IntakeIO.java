package org.frc3512.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {

    double objectDistance = 0.0;
    double temperature = 0;

    boolean hasCoral = false;
    boolean hasAlgae = false;
    boolean isStalled = false;
    boolean maybeHasCoral = false;

    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changeSetpoint(IntakeStates newSetpoint) {}

  public default String getState() {
    return IntakeStates.STOPPED.state;
  }

  public default boolean hasCoral() {
    return false;
  }

  public default boolean maybeHasCoral() {
    return false;
  }

  public default boolean hasAlgae() {
    return false;
  }

  public default boolean isStalled() {
    return false;
  }

  public default double getRed() {
    return 0.0;
  }

  public default double getGreen() {
    return 0.0;
  }

  public default double getBlue() {
    return 0.0;
  }

  public default double getObjectDistance() {
    return 0.0;
  }
}
