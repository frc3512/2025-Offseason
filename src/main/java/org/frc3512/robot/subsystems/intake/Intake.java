package org.frc3512.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeStates desiredState = IntakeStates.STOPPED;

  public static Intake instance;

  public static Intake setInstance(IntakeIO io) {
    instance = new Intake(io);
    return instance;
  }

  public static Intake getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Intake subsystem not initialized yet.");
    }
    return instance;
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public IntakeIO getIntakeIO() {
    return io;
  }

  public void setDesiredState(IntakeStates target) {
    this.desiredState = target;
  }

  @Override
  public void periodic() {

    applyStates();

    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);
  }

  private void applyStates() {
    io.setIntakeSpeed(desiredState.speed);
  }
}
