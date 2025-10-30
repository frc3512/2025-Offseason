package org.frc3512.robot.subsytems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc3512.robot.constants.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {

  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.025, ArmConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1));
  PIDController simController = new PIDController(ArmConstants.kP, 0, 0);

  ArmStates targetPosition = ArmStates.STOW;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAngle = motorSim.getAngularPositionRotations() * 360;
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
