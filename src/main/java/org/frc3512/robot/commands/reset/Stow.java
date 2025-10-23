package org.frc3512.robot.commands.reset;

import org.frc3512.robot.constants.States;
import org.frc3512.robot.subsytems.superstructure.StateMachine;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Stow extends InstantCommand{

    public Stow() {}

    @Override
    public void initialize() {
        new InstantCommand(
            () -> 
                StateMachine.getInstance().applyStates(States.STOWED));
    }
    
}
