package org.frc3512.robot.util;

import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;


public class Position {

    private ElevatorStates wantedElevatorState;
    private ArmStates wantedArmState;
    private WristStates wantedWristState;

    public Position(
        ElevatorStates wantedElevatorState, 
        ArmStates wantedArmState,
        WristStates wantedWristState) {

         this.wantedElevatorState = wantedElevatorState;
         this.wantedArmState = wantedArmState;
         this.wantedWristState = wantedWristState;   

    }

    public ElevatorStates getWantedElevatorState() {
        return wantedElevatorState;
    }

    public ArmStates getWantedArmState() {
        return wantedArmState;
    }

    public WristStates getWantedWristState() {
        return wantedWristState;
    }
    
}
