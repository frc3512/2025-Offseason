package org.frc3512.robot.util;

import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;


public class PositionSetter {

    private ElevatorStates wantedElevatorState;
    private ArmStates wantedArmState;
    private WristStates wantedWristState;

    public PositionSetter(
        ElevatorStates wantedElevatorState, 
        ArmStates wantedArmState,
        WristStates wantedWristState) {

         this.wantedElevatorState = wantedElevatorState;
         this.wantedArmState = wantedArmState;
         this.wantedWristState = wantedWristState;   

    }

    public double getWantedElevatorState() {
        return wantedElevatorState.position;
    }

    public double getWantedArmState() {
        return wantedArmState.position;
    }

    public double getWantedWristState() {
        return wantedWristState.position;
    }
    
}
