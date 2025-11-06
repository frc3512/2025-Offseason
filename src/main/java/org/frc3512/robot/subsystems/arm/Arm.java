package org.frc3512.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3512.robot.constants.Constants.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  public Command changeSetpoint(ArmStates newSetpoint) {
    return runOnce(() -> io.changeSetpoint(newSetpoint));
  }

  @AutoLogOutput(key = "Mechanism States/Arm At Setpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.positionSetpoint - inputs.motorPosition) < ArmConstants.TOLERANCE;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }
}
