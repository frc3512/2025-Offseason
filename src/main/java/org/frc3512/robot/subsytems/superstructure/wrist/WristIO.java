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

        public int state;
    }

    default void setDesiredState(WristStates target)  {}

    default void updateInputs(WristIOInputs inputs) {}

    @Override
    default void refreshData() {}
   
}
