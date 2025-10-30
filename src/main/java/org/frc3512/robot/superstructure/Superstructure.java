package org.frc3512.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Superstructure {

  private StateMachine stateMachine;

  public Superstructure() {

    stateMachine = new StateMachine();
  }

  public Command stow() {
    return new InstantCommand(() -> stateMachine.handleStateTransitions(States.STOW));
  }
}
