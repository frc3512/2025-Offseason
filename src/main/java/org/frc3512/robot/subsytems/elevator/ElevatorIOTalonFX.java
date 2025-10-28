package org.frc3512.robot.subsytems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.frc3512.robot.constants.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX leadMotor;
  private TalonFX followMotor;

  PositionVoltage positionRequest = new PositionVoltage(ElevatorStates.STOW.position);

  private final StatusSignal<Angle> position;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<AngularVelocity> angularVelocity;
  private final StatusSignal<AngularAcceleration> angularAcceleration;

  public ElevatorIOTalonFX() {

    leadMotor = new TalonFX(ElevatorConstants.frontMotorID);
    followMotor = new TalonFX(ElevatorConstants.backMotorID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimit = 80.0;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    config.Slot0.withKP(ElevatorConstants.kP);
    config.Slot0.withKG(ElevatorConstants.kG);

    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

    position = leadMotor.getPosition();
    voltage = leadMotor.getMotorVoltage();
    supplyCurrent = leadMotor.getSupplyCurrent();
    statorCurrent = leadMotor.getStatorCurrent();
    temperature = leadMotor.getDeviceTemp();
    angularVelocity = leadMotor.getRotorVelocity();
    angularAcceleration = leadMotor.getAcceleration();

    leadMotor.getConfigurator().apply(config);
    followMotor.getConfigurator().apply(config);

    leadMotor.setPosition(0.0000000000);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.elevatorHeight = position.getValueAsDouble() * ElevatorConstants.PULLEY_CIRCUMFERENCE;

    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.motorTemp = temperature.getValueAsDouble();
  }

  @Override
  public void setDesiredState(ElevatorStates target) {
    leadMotor.setControl(
        positionRequest.withPosition(target.position / ElevatorConstants.PULLEY_CIRCUMFERENCE));

    followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));
  }

  @Override
  public void refreshData() {
    BaseStatusSignal.refreshAll(
        position,
        voltage,
        supplyCurrent,
        statorCurrent,
        temperature,
        angularVelocity,
        angularAcceleration);
  }
}
