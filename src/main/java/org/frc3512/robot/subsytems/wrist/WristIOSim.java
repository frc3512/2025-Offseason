package org.frc3512.robot.subsytems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc3512.robot.constants.Constants.WristConstants;

public class WristIOSim implements WristIO {

  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.025, WristConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1));
  PIDController simController = new PIDController(WristConstants.kP, 0, 0);

  WristStates targetPosition = WristStates.STOW;

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAngle = motorSim.getAngularPositionRotations() * 360;
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.supplyCurrent = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void updateSim() {
    motorSim.setInputVoltage(
        simController.calculate(
            motorSim.getAngularPositionRotations() * 360, targetPosition.position / 360.0));
    motorSim.update(0.02);
  }

  public double getPosition() {
    return motorSim.getAngularPositionRotations();
  }

  public double getVelocityMetersPerSec() {
    return motorSim.getAngularVelocityRPM() / 60 * 2 * Math.PI;
  }

  public boolean atSetpoint() {
    return Math.abs(motorSim.getAngularPositionRotations() - targetPosition.position / 360.0) < 0.1;
  }
}
