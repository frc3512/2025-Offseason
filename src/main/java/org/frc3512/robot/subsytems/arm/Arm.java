package org.frc3512.robot.subsytems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmStates desiredState = ArmStates.STOW;

  public static Arm instance;

  public static Arm setInstance(ArmIO io) {
    instance = new Arm(io);
    return instance;
  }

  public static Arm getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Arm subsystem not initialized yet.");
    }
    return instance;
  }

  public void setDesiredState(ArmStates target) {
    this.desiredState = target;
  }

  public Arm(ArmIO io) {
    this.io = io;
  }

  public ArmIO getArmIO() {
    return io;
  }

  @Override
  public void periodic() {

    applyStates();

    io.updateInputs(inputs);

    Logger.processInputs("Arm", inputs);
  }

  public void applyStates() {
    io.setDesiredState(desiredState);
  }
}
