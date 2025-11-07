package org.frc3512.robot.subsystems.arm;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.MathUtil;
import org.frc3512.robot.constants.Constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {

  private TalonFX motor;
  private Canandmag encoder;

  private PositionVoltage positionRequest = new PositionVoltage(ArmStates.STOW.degrees);

  private double desiredState;

  public ArmIOTalonFX() {

    motor = new TalonFX(15);
    encoder = new Canandmag(30);

    motor.getConfigurator().apply(ArmConstants.config);

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

    motor.setPosition(getAbsEncoderDeg() / 360.0);
  }

  // Zeros the arm
  public double getAbsEncoderDeg() {
    return ((360.0 * encoder.getAbsPosition() + 180) % 360.0) - 180.0;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    motor.setControl(positionRequest.withPosition(desiredState / 360.0));

    // Update inputs
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        motor.getPosition().getValueAsDouble() * 360; // Convert rotations to degrees
    inputs.motorVelocity = motor.getVelocity().getValueAsDouble() * 360; // Convert rps to dps
    inputs.motorAcceleration =
        motor.getAcceleration().getValueAsDouble() * 360; // Convert rps^2 to dps^2
    inputs.positionSetpoint = positionRequest.Position * 360; // Convert rotations to degrees
  }

  @Override
  public void changeSetpoint(ArmStates newSetpoint) {
    desiredState = MathUtil.clamp(newSetpoint.degrees, -123, 123);
  }
}
