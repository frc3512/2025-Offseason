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
    
    // * Coral Positions

    // | Intake Coral Position
    public static final Position INTAKING_CORAL =
        new Position(
            ArmStates.INTAKE_CORAL,
                ElevatorStates.INTAKE,
                    WristStates.INTAKE);

    // | Prep Coral Positions
    public static final Position PREP_CORAL = 
        new Position(
            ArmStates.HOLD_CORAL,
                ElevatorStates.PREP_CORAL, 
                    WristStates.CORAL);

    public static final Position PREP_L2 = 
        new Position(
            ArmStates.PREP_MID,
                ElevatorStates.L2,
                    WristStates.CORAL);

    public static final Position PREP_L3 = 
        new Position(
            ArmStates.PREP_MID,
                ElevatorStates.L3,
                    WristStates.CORAL);   

    public static final Position PREP_L4 = 
        new Position(
            ArmStates.PREP_L4,
                ElevatorStates.L4,
                    WristStates.CORAL);

    // | Place Coral Positions  
    public static final Position PLACE_L2 = 
        new Position(
            ArmStates.PLACE_MID,
                ElevatorStates.L2,
                    WristStates.CORAL);

    public static final Position PLACE_L3 = 
        new Position(
            ArmStates.PLACE_MID,
                ElevatorStates.L3,
                    WristStates.CORAL);

    public static final Position PLACE_L4 =
        new Position(
            ArmStates.PLACE_L4,
                ElevatorStates.L4,
                    WristStates.CORAL);

    // | L1
    public static final Position PREP_TROUGH = 
        new Position(
            ArmStates.TROUGH,
                ElevatorStates.TROUGH,
                    WristStates.TROUGH);

}