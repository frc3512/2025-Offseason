package org.frc3512.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

  private LedIO io;
  private LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public Led(LedIO io) {
    this.io = io;
  }


  public Command setPattern(LedStates state) {
    return Commands.runOnce(() -> io.setPattern(state));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);
  }

}
