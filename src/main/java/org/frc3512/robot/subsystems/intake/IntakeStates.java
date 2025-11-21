package org.frc3512.robot.subsystems.intake;

public enum IntakeStates {
  INTAKE(-0.5, "Intaking"),

  EJECT(0.9, "Ejecting Algae"),

  SPIT(0.4, "Spitting Coral"),

  PLACE(0.15, "Placing Coral"),

  HOLD(-0.2, "Holding Algae"),

  STOPPED(0.0, "Stopped");

  public double speed;
  public String state;

  IntakeStates(double wantedSpeed, String state) {
    this.speed = wantedSpeed;
    this.state = state;
  }
}
