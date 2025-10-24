package org.frc3512.robot.subsytems.superstructure.elevator;

import org.frc3512.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO extends SubsystemDataProcessor.IODataRefresher{

    @AutoLog
    class ElevatorIOInputs {

        public double elevatorHeight;

        public double appliedVolts;
        public double supplyCurrent;
        public double statorCurrent;
        public double motorTemp;

        public int state;
    }

    default void setDesiredState(ElevatorStates target)  {}

    default void updateInputs(ElevatorIOInputs inputs) {}

    @Override
    default void refreshData() {}
   
}
