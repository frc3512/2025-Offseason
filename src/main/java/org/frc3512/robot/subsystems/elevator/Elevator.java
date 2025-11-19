package org.frc3512.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3512.robot.constants.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Command changeSetpoint(ElevatorStates newSetpoint) {
    return runOnce(() -> io.changeSetpoint(newSetpoint));
  }

  @AutoLogOutput(key = "Mechanism States/ElevatorAtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.positionSetpoint - inputs.motorPosition)
        < ElevatorConstants.heightTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
