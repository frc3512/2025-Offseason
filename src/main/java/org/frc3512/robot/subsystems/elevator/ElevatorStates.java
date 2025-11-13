package org.frc3512.robot.subsystems.elevator;

public enum ElevatorStates {

  // * Values are in inches, conversion happens in Elevator.java

  // * Defualt
  STOW(0.0, "Stowed"),

  // * Coral

  // | Reef
  PREP_L2(3, "Prepping L2"),
  PREP_L3(18, "Prepping L3"),
  PREP_l4(46, "Prepping L4"),

  PLACE_L2(2, "Placing L2"),
  PLACE_L3(17, "Placing L3"),
  PLACE_L4(45, "Placing L4"),

  TROUGH(7, "Trough"),

  // | Intake
  INTAKE(11, "Intaking Coral"),
  PREP_CORAL(5, "Preping Coral"),

  // * Algae

  // | De-Reef
  ALGAE_L1(15, "Algae L1"),
  ALGAE_L2(28, "Algae L2"),

  // | Intake
  ALGAE_INTAKE(11, "Intaking Algae"),

  // | Prep
  PREP_ALGAE(9, "Preping Algae"),

  // | Barge
  BARGE(60, "Barging"),

  // | Processor
  PROCESSOR(8, "Processing"),

  // * Testing
  TEST_1(25, "Testing");

  public final double position;
  public final String state;

  ElevatorStates(double totalPosition, String state) {
    this.position = totalPosition;
    this.state = state;
  }
}
