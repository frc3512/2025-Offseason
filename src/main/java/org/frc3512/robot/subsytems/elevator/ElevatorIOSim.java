package org.frc3512.robot.subsytems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc3512.robot.constants.Constants.ArmConstants;
import org.frc3512.robot.constants.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.025, ArmConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  PIDController simController = new PIDController(ElevatorConstants.kP, 0, 0);

  ElevatorStates targetPosition = ElevatorStates.STOW;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorHeight =
        motorSim.getAngularPositionRotations() * ElevatorConstants.PULLEY_CIRCUMFERENCE;
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.supplyCurrent = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void updateSim() {
    motorSim.setInputVoltage(
        simController.calculate(
            motorSim.getAngularPositionRotations() * ElevatorConstants.PULLEY_CIRCUMFERENCE,
            targetPosition.position / ElevatorConstants.PULLEY_CIRCUMFERENCE));
    motorSim.update(0.02);
  }

  public double getVelocityMetersPerSec() {
    return motorSim.getAngularVelocityRPM() / 60 * 2 * Math.PI;
  }

  public boolean atSetpoint() {
    return Math.abs(
            motorSim.getAngularPositionRotations()
                - targetPosition.position / ElevatorConstants.PULLEY_CIRCUMFERENCE)
        < 0.1;
  }
}
