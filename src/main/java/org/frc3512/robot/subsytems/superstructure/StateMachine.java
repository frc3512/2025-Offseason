package org.frc3512.robot.subsytems.superstructure;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;

public class StateMachine {

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

    public StateMachine(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO) {}

    public void applyStates() {}

}