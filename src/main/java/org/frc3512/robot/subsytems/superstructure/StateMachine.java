package org.frc3512.robot.subsytems.superstructure;

import org.frc3512.robot.constants.States;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;
import org.frc3512.robot.util.Position;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase{

    private Position wantedPosition;

    private final ArmIO armIO;
    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private static StateMachine instance;

    public static StateMachine getInstance() {
        if (instance == null) {
            throw new IllegalStateException("State Machine not initialized yet.");
        }
        return instance;
    }

    public static StateMachine setInstance(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO) {
        instance = new StateMachine(armIO, elevatorIO, wristIO);
        return instance;
    }

    public StateMachine(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO) {

        this.armIO = armIO;
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;

        wantedPosition = States.STOWED;

    }

    @Override
    public void periodic() {
        applyStates(wantedPosition);
    }

    public void applyStates(Position position) {

        armIO.setDesiredState(position.getWantedArmState());
        elevatorIO.setDesiredState(position.getWantedElevatorState());
        wristIO.setDesiredState(position.getWantedWristState());

    }

}