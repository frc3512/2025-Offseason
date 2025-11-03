package org.frc3512.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc3512.robot.constants.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {

  private TalonFX motor;
  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0); // Mechanism Rotations

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(ArmConstants.simMotor, 0.001, ArmConstants.GEAR_RATIO),
          ArmConstants.simMotor);

  public ArmIOSim() {

    // Motor Initialization
    motor = new TalonFX(ArmConstants.ID);
    motor.getConfigurator().apply(ArmConstants.config);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    // Apply setpoints
    motor.setControl(setpoint);

    // Simulate motor
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(0.02);
    motorSim.setRawRotorPosition(motorModel.getAngularPosition().times(ArmConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(ArmConstants.GEAR_RATIO));

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
  public void changeSetpoint(ArmStates newSetpoint) {
    var degrees = MathUtil.clamp(newSetpoint.degrees, -120, 120);
    setpoint.Position = degrees / 360.0; // Convert degrees to rotations
  }
}
