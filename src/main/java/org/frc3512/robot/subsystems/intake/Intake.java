package org.frc3512.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command changeSetpoint(IntakeStates setpoint) {
    return runOnce(() -> io.changeSetpoint(setpoint));
  }

  public String getState() {
    return io.getState();
  }

  public boolean hasCoral() {
    return io.hasCoral();
  }

  public boolean maybeHasCoral() {
    return io.maybeHasCoral();
  }

  public boolean hasAlgae() {
    return io.hasAlgae();
  }

  public boolean isStalled() {
    return io.isStalled();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
