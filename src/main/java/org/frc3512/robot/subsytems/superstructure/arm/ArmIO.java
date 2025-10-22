package org.frc3512.robot.subsytems.superstructure.arm;

import org.frc3512.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO extends SubsystemDataProcessor.IODataRefresher{

    @AutoLog
    public class ArmIOInputs {

        public double armAngle;

        public double appliedVolts;
        public double supplyCurrent;
        public double statorCurrent;
        public double motorTemp;

    }

    default void setDesiredState(ArmStates target)  {}

    default void updateInputes(ArmIOInputs inputs) {}

    default double getPosition() {return 0.0;} 

    @Override
    default void refreshData() {}
   
}
