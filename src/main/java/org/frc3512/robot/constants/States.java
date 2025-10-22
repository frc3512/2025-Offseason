package org.frc3512.robot.constants;

import org.frc3512.robot.subsytems.superstructure.arm.ArmStates;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.superstructure.wrist.WristStates;
import org.frc3512.robot.util.Position;

public class States {

    // * Defualt Position
    public static final Position STOWED = 
        new Position(
            ArmStates.STOW,
                ElevatorStates.STOW,
                    WristStates.STOW);

}