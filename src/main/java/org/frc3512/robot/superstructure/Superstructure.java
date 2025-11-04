package org.frc3512.robot.superstructure;

import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Superstructure {

  // private StateMachine stateMachine;

  Elevator elevator;
  Arm arm;

  public Superstructure(Elevator elevator, Arm arm) {

    // stateMachine = new StateMachine();

    this.elevator = elevator;
    this.arm = arm;
  }

  public Command stow() {
    return Commands.parallel(elevator.changeSetpoint(ElevatorStates.STOW));
  }
}
