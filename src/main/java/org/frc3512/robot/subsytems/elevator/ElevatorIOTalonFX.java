package org.frc3512.robot.subsytems.elevator;

import org.frc3512.robot.constants.Constants.ElevatorConstants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX leadMotor;
  private TalonFX followMotor;

  PositionVoltage setpoint = new PositionVoltage(ElevatorStates.STOW.position);

  public ElevatorIOTalonFX() {

    // Motor Initialization
    leadMotor = new TalonFX(ElevatorConstants.frontMotorID);
    followMotor = new TalonFX(ElevatorConstants.backMotorID);

    leadMotor.getConfigurator().apply(ElevatorConstants.config);
    followMotor.getConfigurator().apply(ElevatorConstants.config);

    // Optimize Can Bus Utilization
    var motorVoltageSignal = leadMotor.getMotorVoltage();
    var motorCurrentSignal = leadMotor.getSupplyCurrent();
    var motorPositionSignal = leadMotor.getPosition();
    var motorVelocitySignal = leadMotor.getVelocity();
    var motorAccelerationSignal = leadMotor.getAcceleration();

    motorVoltageSignal.setUpdateFrequency(50);
    motorCurrentSignal.setUpdateFrequency(50);
    motorPositionSignal.setUpdateFrequency(50);
    motorVelocitySignal.setUpdateFrequency(50);
    motorAccelerationSignal.setUpdateFrequency(50);

    leadMotor.optimizeBusUtilization();
    followMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    leadMotor.setControl(setpoint);

    followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));

    // Update inputs
    inputs.motorVoltage = leadMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = leadMotor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        leadMotor.getPosition().getValueAsDouble()
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rotations to inches
    inputs.motorVelocity =
        leadMotor.getVelocity().getValueAsDouble()
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rps to ips
    inputs.motorAcceleration =
        leadMotor.getAcceleration().getValueAsDouble()
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rps^2 to ips^2
    inputs.positionSetpoint =
        setpoint.Position * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rotations to inches
  }

  @Override
  public void changeSetpoint(ElevatorStates newSetpoint) {
    var inches =
        MathUtil.clamp(
            newSetpoint.position, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    setpoint.Position =
        inches / ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert degrees to rotations
  }
}
