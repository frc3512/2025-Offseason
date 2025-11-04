package org.frc3512.robot.subsystems.wrist;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import org.frc3512.robot.constants.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {

  private TalonFX motor;

  PositionVoltage setpoint = new PositionVoltage(WristStates.STOW.degrees);

  public WristIOTalonFX() {

    motor = new TalonFX(15);

    motor.getConfigurator().apply(WristConstants.config);

    var motorVoltageSignal = motor.getMotorVoltage();
    var motorCurrentSignal = motor.getSupplyCurrent();
    var motorPositionSignal = motor.getPosition();
    var motorVelocitySignal = motor.getVelocity();
    var motorAccelerationSignal = motor.getAcceleration();

    motorVoltageSignal.setUpdateFrequency(50);
    motorCurrentSignal.setUpdateFrequency(50);
    motorPositionSignal.setUpdateFrequency(50);
    motorVelocitySignal.setUpdateFrequency(50);
    motorAccelerationSignal.setUpdateFrequency(50);

    motor.setPosition(0.00000000);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    motor.setControl(setpoint);

    // Update inputs
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        motor.getPosition().getValueAsDouble() * 360; // Convert rotations to degrees
    inputs.motorVelocity = motor.getVelocity().getValueAsDouble() * 360; // Convert rps to dps
    inputs.motorAcceleration =
        motor.getAcceleration().getValueAsDouble() * 360; // Convert rps^2 to dps^2
    inputs.positionSetpoint = setpoint.Position * 360; // Convert rotations to degrees
  }

  @Override
  public void changeSetpoint(WristStates newSetpoint) {
    var degrees = MathUtil.clamp(newSetpoint.degrees, -5, 95);
    setpoint.Position = degrees / 360.0; // Convert degrees to rotations
  }
}
