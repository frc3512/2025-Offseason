package org.frc3512.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.Timer;
import org.frc3512.robot.constants.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX motor;
  private final Canandcolor sensor;

  private DutyCycleOut setpoint = new DutyCycleOut(0);

  private double currentTime = 0.0;
  private IntakeStates currentState = IntakeStates.STOPPED;

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
    inputs.objectDistance = getObjectDistance();
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();

    inputs.hasAlgae = hasAlgae();
    inputs.hasCoral = hasCoral();
    inputs.isStalled = isStalled();

    inputs.red = sensor.getRed();
    inputs.green = sensor.getGreen();
    inputs.blue = sensor.getBlue();
  }

  @Override
  public void changeSetpoint(IntakeStates newSetpoint) {
    setpoint.Output = newSetpoint.speed; // Set desired voltage
  }

  @Override
  public String getState() {
    return currentState.state;
  }

  @Override
  public double getObjectDistance() {
    return sensor.getProximity();
  }

  @Override
  public boolean hasCoral() {
    if (getObjectDistance() <= 0.04) {
      if (getRed() >= 0
          && getRed() <= 0.06
          && getGreen() >= 0
          && getGreen() <= 0.06
          && getBlue() >= 0
          && getBlue() <= 0.06) {

        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  @Override
  public boolean maybeHasCoral() {
    if (getObjectDistance() <= 0.05 && !hasAlgae()) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public boolean hasAlgae() {
    if (getObjectDistance() <= 0.05) {
      if (getRed() >= 0.05
          && getRed() <= 0.15
          && getGreen() >= 0.29
          && getGreen() <= 0.36
          && getBlue() >= 0.11
          && getBlue() <= 0.17) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  @Override
  public boolean isStalled() {
    if (motor.getStatorCurrent().getValueAsDouble() < 30) {
      currentTime = Timer.getFPGATimestamp();
      return false;
    } else if (motor.getStatorCurrent().getValueAsDouble() > 30
        && Timer.getFPGATimestamp() - currentTime > 0.2) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public double getRed() {
    return sensor.getRed();
  }

  @Override
  public double getGreen() {
    return sensor.getGreen();
  }

  @Override
  public double getBlue() {
    return sensor.getBlue();
  }
}
