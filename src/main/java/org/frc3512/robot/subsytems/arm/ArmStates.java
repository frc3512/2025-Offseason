package org.frc3512.robot.subsytems.arm;

public enum ArmStates {

  // * Units are in degrees
  // negative = forward

  // * Defualt
  STOW(0.0, "Stowed"),

  // * Coral

  // | Score
  PREP_MID(-28, "Preping Mid"),
  PLACE_MID(-65, "Placing Mid"),

  PREP_L4(-32, "Preping L4"),
  PLACE_L4(-60, "Placing L4"),

  TROUGH(-75, "Trough"),

  // | Intake
  INTAKE_CORAL(117, "Intaking Coral"),
  INTAKE_ALGAE(80, "Intaking Algae"),

  // | Prep
  HOLD_CORAL(-10, "Holding Coral"),

  // * Algae

  // | De-Reef
  REMOVE_ALGAE(-65, "Removing Algae"),

  // | Prep
  PREP_ALGAE(0.0, "Preping Algae"),

  // | Process
  PROCESS(-75, "Processing"),

  // | Barge
  BARGE(15, "Barging"),

  // * Testing
  FRONT(-40, "Front"),
  BACK(40, "Back");

  public double position;
  public String state;

  ArmStates(double totalPosition, String state) {
    this.position = totalPosition;
    this.state = state;
  }
}
