package org.frc3512.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc3512.robot.constants.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

  private TalonFX leadMotor;
  private TalonFX followMotor;

  PositionVoltage setpoint = new PositionVoltage(ElevatorStates.STOW.position);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ElevatorConstants.simMotor, 0.001, ElevatorConstants.GEAR_RATIO),
          ElevatorConstants.simMotor);

  public ElevatorIOSim() {

    // Motor Initialization
    leadMotor = new TalonFX(ElevatorConstants.frontMotorID);
    followMotor = new TalonFX(ElevatorConstants.backMotorID);

    leadMotor.getConfigurator().apply(ElevatorConstants.config);
    followMotor.getConfigurator().apply(ElevatorConstants.config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    leadMotor.setControl(setpoint);

    followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));

    // Simulate motor
    var motorSim = leadMotor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(0.02);
    motorSim.setRawRotorPosition(
        motorModel.getAngularPosition().times(ElevatorConstants.GEAR_RATIO));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(ElevatorConstants.GEAR_RATIO));

    // Update inputs
    inputs.motorVoltage = leadMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = leadMotor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        leadMotor.getPosition().getValueAsDouble()
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rotations to vertical inches
    inputs.motorVelocity =
        leadMotor.getVelocity().getValueAsDouble()
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rps to inches per second
    inputs.motorAcceleration =
        leadMotor.getAcceleration().getValueAsDouble()
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rps^2 to inches per second squared
    inputs.positionSetpoint =
        setpoint.Position
            * ElevatorConstants.PULLEY_CIRCUMFERENCE; // Convert rotations to vertical inches
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
