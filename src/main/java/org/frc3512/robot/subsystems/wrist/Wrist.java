package org.frc3512.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3512.robot.constants.Constants.WristConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  public Command changeSetpoint(WristStates newSetpoint) {
    return runOnce(() -> io.changeSetpoint(newSetpoint));
  }

  @AutoLogOutput(key = "Mechanism States/Wrist At Setpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.positionSetpoint - inputs.motorPosition) 
      < WristConstants.TOLERANCE;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist/", inputs);
  }
}
