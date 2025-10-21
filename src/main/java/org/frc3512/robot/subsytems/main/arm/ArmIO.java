package org.frc3512.robot.subsytems.main.arm;

import org.frc3512.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO extends SubsystemDataProcessor.IODataRefresher{

    @AutoLog
    class ArmIOInputs {

        public double armAngle;

        public double appliedVolts;
        public double supplyCurrent;
        public double statorCurrent;
        public double angularVelocity;
        public double angularAcceleration;
        public double motorTemp;

    }

    default void setDesiredState(ArmStates target)  {}

    default void updateInputes(ArmIOInputs inputs) {}

    @Override
    default void refreshData() {}
   
}
