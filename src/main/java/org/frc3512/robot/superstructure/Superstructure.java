package org.frc3512.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.arm.ArmStates;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeStates;
import org.frc3512.robot.subsystems.wrist.Wrist;
import org.frc3512.robot.subsystems.wrist.WristStates;

public class Superstructure {

  // private StateMachine stateMachine;

  Arm arm;
  Elevator elevator;
  Wrist wrist;

  Intake intake;

  public Superstructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake) {

    // stateMachine = new StateMachine();

    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;

    this.intake = intake;
  }

  public Command stow() {
    return Commands.parallel(
      arm.changeSetpoint(ArmStates.STOW),
      elevator.changeSetpoint(ElevatorStates.STOW),
      wrist.changeSetpoint(WristStates.STOW),
      
      intake.changeSetpoint(IntakeStates.STOPPED));
  }
}
