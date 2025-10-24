package org.frc3512.robot.subsytems.intake;

public enum IntakeStates {

    INTAKE(-0.5),

    EJECT(0.9),

    PLACE(0.15),

    HOLD(0.25),

    STOPPED(0.0);
    
    double speed;

    IntakeStates(double wantedSpeed) {
        this.speed = wantedSpeed;
    }
    
}
