package org.frc3512.robot.commands.reset;

import org.frc3512.robot.constants.States;
import org.frc3512.robot.subsytems.superstructure.StateMachine;
import org.frc3512.robot.subsytems.superstructure.arm.Arm;
import org.frc3512.robot.subsytems.superstructure.arm.ArmIO;
import org.frc3512.robot.subsytems.superstructure.elevator.Elevator;
import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO;
import org.frc3512.robot.subsytems.superstructure.wrist.Wrist;
import org.frc3512.robot.subsytems.superstructure.wrist.WristIO;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Stow extends InstantCommand{

    private final ArmIO armIO;
    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private StateMachine stateMachine;

    public Stow() {

        this.armIO = Arm.getInstance().getArmIO();
        this.elevatorIO = Elevator.getInstance().getElevatorIO();
        this.wristIO = Wrist.getInstance().getWristIO();

        stateMachine = new StateMachine(armIO, elevatorIO, wristIO);

    }

    @Override
    public void initialize() {
        new InstantCommand(() -> stateMachine.applyStates(States.STOWED));
    }
    
}
