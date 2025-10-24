package org.frc3512.robot.subsytems.superstructure;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;

@SuppressWarnings("unused")
public class StateMachine {

    private States wantedState = States.STOW;

    private ArmIO armIO;
    private ElevatorIO elevatorIO;
    private WristIO wristIO;

    public StateMachine(ArmIO armIO, ElevatorIO elevatorIO, WristIO wristIO) {

        this.armIO = armIO;
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;

    }

    public void applyStates() {}

}