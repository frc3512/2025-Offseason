package org.frc3512.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Wrist extends SubsystemBase {

  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private WristStates desiredState = WristStates.STOW;

  public static Wrist instance;

  public static Wrist setInstance(WristIO io) {
    instance = new Wrist(io);
    return instance;
  }

  public static Wrist getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Wrist subsystem not initialized yet.");
    }
    return instance;
  }

  public void setDesiredState(WristStates target) {
    this.desiredState = target;
  }

  public Wrist(WristIO io) {
    this.io = io;
  }

  public WristIO getWristIO() {
    return io;
  }

  @Override
  public void periodic() {

    applyStates();

    io.updateInputs(inputs);

    Logger.processInputs("Wrist", (LoggableInputs) inputs);
  }

  public void applyStates() {
    io.setDesiredState(desiredState);
  }
}
