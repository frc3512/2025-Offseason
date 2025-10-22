package org.frc3512.robot.util;

import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;


public class Position {

    private ArmStates wantedArmState;
    private ElevatorStates wantedElevatorState;
    private WristStates wantedWristState;

    public Position(
        ArmStates wantedArmState,
        ElevatorStates wantedElevatorState, 
        WristStates wantedWristState) {

        this.wantedArmState = wantedArmState;
        this.wantedElevatorState = wantedElevatorState;
        this.wantedWristState = wantedWristState;   
    }
   
    public ArmStates getWantedArmState() {
        return wantedArmState;
    }
    
    public ElevatorStates getWantedElevatorState() {
        return wantedElevatorState;
    }

    public WristStates getWantedWristState() {
        return wantedWristState;
    }
    
}
