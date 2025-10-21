package org.frc3512.robot.subsytems.superstructure.elevator;

public enum ElevatorStates {

    // * Values are in inches, conversion happens in Elevator.java

    // * Defualt
    STOW(0.0, "Stowed"),
 
    // * Coral

    // | Reef
    L4(46, "L4"),
    L3(18, "L3"),
    L2(0, "L2"),

    TROUGH(7, "Trough"),

    // | Intake
    INTAKE(11, "Intaking Coral"),
    PREP_CORAL(0, "Preping Coral"),

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

    // * Testing
    TEST_1(25, "Testing");

    public final double position;
    public final String state;

    ElevatorStates(double totalPosition, String state) {
        this.position = totalPosition;
        this.state = state;
    }

}
