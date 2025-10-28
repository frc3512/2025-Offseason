package org.frc3512.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.frc3512.robot.subsytems.arm.Arm;
import org.frc3512.robot.subsytems.elevator.Elevator;
import org.frc3512.robot.subsytems.intake.Intake;
import org.frc3512.robot.subsytems.wrist.Wrist;

public class Superstructure {

  private StateMachine stateMachine;

  public Superstructure() {

    stateMachine =
        new StateMachine(
            Arm.getInstance().getArmIO(),
            Elevator.getInstance().getElevatorIO(),
            Wrist.getInstance().getWristIO(),
            Intake.getInstance().getIntakeIO());
  }

  public Command stow() {
    return new InstantCommand(() -> stateMachine.handleStateTransitions(States.STOW));
  }
}
