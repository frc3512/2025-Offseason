package org.frc3512.robot.subsytems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motor;
  private final TalonFXConfiguration config;

  private final Canandcolor sensor;
  private final CanandcolorSettings settings;

  public IntakeIOTalonFX() {

    motor = new TalonFX(17);

    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    sensor = new Canandcolor(31);
    settings = new CanandcolorSettings();

    settings.setColorFramePeriod(0.040);
    settings.setLampLEDBrightness(0.35);
    settings.setAlignColorFramesToIntegrationPeriod(true);
    settings.setProximityIntegrationPeriod(ProximityPeriod.k20ms);

    motor.getConfigurator().apply(config);
    sensor.setSettings(settings);
  }

  public double getObjectDistance() {
    return sensor.getProximity();
  }

  @Override
  public boolean hasCoral() {
    if (getObjectDistance() <= 0.04) {
      if (getR() >= 0
          && getR() <= 0.06
          && getG() >= 0
          && getG() <= 0.06
          && getB() >= 0
          && getB() <= 0.06) {

        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  // fix-me
  @Override
  public boolean hasAlgae() {
    if (getObjectDistance() <= 0.05) {
      if (getR() >= 0.07
          && getR() <= 0.13
          && getG() >= 0.27
          && getG() <= 0.34
          && getB() >= 0.09
          && getB() <= 0.15) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public ColorData getColor() {
    return sensor.getColor();
  }

  public double getR() {
    return sensor.getRed();
  }

  public double getG() {
    return sensor.getGreen();
  }

  public double getB() {
    return sensor.getBlue();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.objectDistance = getObjectDistance();

    inputs.hasCoral = hasCoral();
    inputs.hasAlgae = hasAlgae();

    inputs.red = getR();
    inputs.green = getG();
    inputs.blue = getB();
  }

  @Override
  public void setDesiredState(IntakeStates target) {
    setSpeed(target.speed);
  }
}
