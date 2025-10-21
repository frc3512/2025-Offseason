package org.frc3512.robot.subsytems.superstructure.wrist;

import org.frc3512.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO extends SubsystemDataProcessor.IODataRefresher{

    @AutoLog
    class WristIOInputs {

        public double wristAngle;

        public double appliedVolts;
        public double supplyCurrent;
        public double statorCurrent;
        public double motorTemp;

    }

    default void setDesiredState(WristStates target)  {}

    default void updateInputes(WristIOInputs inputs) {}

    @Override
    default void refreshData() {}
   
}
