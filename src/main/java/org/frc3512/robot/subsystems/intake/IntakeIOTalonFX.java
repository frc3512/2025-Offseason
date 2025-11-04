package org.frc3512.robot.subsystems.intake;

import org.frc3512.robot.constants.Constants.IntakeConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX motor;
  private final Canandcolor sensor;

  private DutyCycleOut setpoint = new DutyCycleOut(0);

  public IntakeIOTalonFX() {

    // Motor Initialization
    motor = new TalonFX(IntakeConstants.motorID);
    sensor = new Canandcolor(IntakeConstants.sensorID);

    motor.getConfigurator().apply(IntakeConstants.config);
    sensor.setSettings(IntakeConstants.sensorConfig);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Apply setpoints
    motor.setControl(setpoint);

    // Update inputs
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(IntakeStates newSetpoint) {
    setpoint.Output = newSetpoint.speed; // Set desired voltage
  }

}