package org.frc3512.robot.subsytems.main.arm;

import org.frc3512.robot.constants.Constants.ArmConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class Arm implements ArmIO{

    private TalonFX motor;
    private Canandmag encoder;

    PositionVoltage positionRequest = new PositionVoltage(ArmStates.STOW.position);

    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<AngularVelocity> angularVelocity;
    private final StatusSignal<AngularAcceleration> angularAcceleration;

    public Arm() {

        motor = new TalonFX(16);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = ArmConstants.GEAR_RATIO;

        config.Slot0.withKP(ArmConstants.kP);
        config.Slot0.withKD(ArmConstants.kD);
        config.Slot0.withKA(ArmConstants.kA);

        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        temperature = motor.getDeviceTemp();
        angularVelocity = motor.getRotorVelocity();
        angularAcceleration = motor.getAcceleration();

        motor.setPosition(getAbsEncoderDeg() / 360.0);

    }

    // Zeros the arm
    public double getAbsEncoderDeg() {
        return ((360.0 * encoder.getAbsPosition() + 180 ) % 360.0) - 180.0; 
    }

    @Override
    public void updateInputes(ArmIOInputs inputs) {

        inputs.armAngle = position.getValueAsDouble() * 360;

        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.motorTemp = temperature.getValueAsDouble();

        inputs.angularVelocity =
            angularVelocity.getValueAsDouble() * ArmConstants.ARM_POSITION_COEFFICIENT;

        inputs.angularAcceleration =
            angularAcceleration.getValueAsDouble() * ArmConstants.ARM_POSITION_COEFFICIENT;

    }

    @Override
    public void setDesiredState(ArmStates target) {
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
