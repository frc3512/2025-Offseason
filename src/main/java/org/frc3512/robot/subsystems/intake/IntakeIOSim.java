package org.frc3512.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc3512.robot.constants.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private TalonFX motor;
  private VoltageOut setpoint = new VoltageOut(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              IntakeConstants.simMotor, 0.0001, IntakeConstants.GEAR_RATIO),
          IntakeConstants.simMotor);

  public IntakeIOSim() {

    // Motor Initialization
    motor = new TalonFX(IntakeConstants.motorID);
    motor.getConfigurator().apply(IntakeConstants.config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Apply setpoints
    motor.setControl(setpoint);

    // Simulate motor
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(0.02);
    motorSim.setRawRotorPosition(motorModel.getAngularPosition().times(IntakeConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(IntakeConstants.GEAR_RATIO));

    // Update inputs
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(IntakeStates newSetpoint) {
    setpoint.Output = newSetpoint.speed;
  }
}
