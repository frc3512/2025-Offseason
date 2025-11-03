package org.frc3512.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;

public class Superstructure {

  // private StateMachine stateMachine;

  Elevator elevator;

  public Superstructure(Elevator elevator) {

    // stateMachine = new StateMachine();

    this.elevator = elevator;
  }

  public Command stow() {
    return Commands.parallel(elevator.changeSetpoint(ElevatorStates.STOW));
  }
}
