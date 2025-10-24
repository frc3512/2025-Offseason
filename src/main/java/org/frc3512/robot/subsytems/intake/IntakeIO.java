package org.frc3512.robot.subsytems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {

        double objectDistance = 0.0;
        
        double velocityRPM = 0.0;
        double appliedVolts = 0.0;
        double currentAmps = 0.0;
        boolean hasCoral = false;
        boolean hasAlgae = false;
        boolean isIntaking = false;
        double temperature = 0;

        double red = 0.0;
        double green = 0.0;
        double blue = 0.0;

    }

    default void setIntakeSpeed(double speed) {}

    default void updateInputs(IntakeIOInputs inputs) {}

    default boolean hasCoral() {
        return false;
    }

    default boolean hasAlgae() {
        return false;
    }

    default boolean isIntaking() {
        return false;
    }

    default void setDesiredState(IntakeStates target) {}
    
}