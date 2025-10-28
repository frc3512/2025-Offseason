package org.frc3512.robot.subsytems.wrist;

public enum WristStates {

  // * Positions in degrees

  // | HORIZONTAL
  INTAKE(90, "Intaking"),
  ALGAE(90, "Holding Algae"),
  TROUGH(90, "Placing Trough"),

  // | VERTICAL
  CORAL(0, "Holding Coral"),
  STOW(0, "Stowed"),

  // DIAGONAL
  PROCESS(45, "Processing");

  public double position;
  public String state;

  WristStates(double totalPosition, String state) {
    this.position = totalPosition;
    this.state = state;
  }
}
