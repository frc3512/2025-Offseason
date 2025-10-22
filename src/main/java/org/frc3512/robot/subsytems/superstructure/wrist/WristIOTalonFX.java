package org.frc3512.robot.subsytems.superstructure.wrist;

import org.frc3512.robot.constants.Constants.WristConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class WristIOTalonFX implements WristIO{

    private TalonFX motor;

    PositionVoltage positionRequest = new PositionVoltage(WristStates.STOW.position);

    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<AngularVelocity> angularVelocity;
    private final StatusSignal<AngularAcceleration> angularAcceleration;

    public WristIOTalonFX() {

        motor = new TalonFX(16);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimit = 20.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = WristConstants.GEAR_RATIO;

        config.Slot0.withKP(WristConstants.kP);

        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        temperature = motor.getDeviceTemp();
        angularVelocity = motor.getRotorVelocity();
        angularAcceleration = motor.getAcceleration();

        motor.setPosition(0.000000);

    }

    @Override
    public void updateInputes(WristIOInputs inputs) {

        inputs.wristAngle = position.getValueAsDouble() * 360;

        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.motorTemp = temperature.getValueAsDouble();

    }

    @Override
    public void setDesiredState(WristStates target) {
        motor.setControl(
            positionRequest.withPosition(target.position / 360.0));

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
            angularAcceleration
        );
    }
    
}
