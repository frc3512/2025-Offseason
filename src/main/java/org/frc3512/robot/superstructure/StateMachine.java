package org.frc3512.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3512.robot.subsytems.arm.Arm;
import org.frc3512.robot.subsytems.arm.ArmStates;
import org.frc3512.robot.subsytems.elevator.Elevator;
import org.frc3512.robot.subsytems.elevator.ElevatorStates;
import org.frc3512.robot.subsytems.intake.Intake;
import org.frc3512.robot.subsytems.intake.IntakeStates;
import org.frc3512.robot.subsytems.wrist.Wrist;
import org.frc3512.robot.subsytems.wrist.WristStates;

public class StateMachine extends SubsystemBase {

  private States currentState;

  private Arm arm = Arm.getInstance();
  private ArmStates currentArmState;
  private Elevator elevator = Elevator.getInstance();
  private ElevatorStates currentElevatorState;
  private Wrist wrist = Wrist.getInstance();
  private WristStates currentWristState;
  private Intake intake = Intake.getInstance();
  private IntakeStates currentIntakeState;

  // Add booleans to keep track of if we have preped a socriong state yet
  Boolean preppedTrough = false;
  Boolean preppedL2 = false;
  Boolean preppedL3 = false;
  Boolean preppedL4 = false;

  Boolean preppedBarge = false;
  Boolean preppedProcessor = false;

  public StateMachine() {}

  public void handleStateTransitions(States wantedState) {

    switch (wantedState) {
      case STOW:
        if (!intake.getIntakeIO().hasAlgae() && !intake.getIntakeIO().hasCoral()) {
          currentState = States.STOW;
        }
      case INTAKE_CORAL:
        if (!intake.getIntakeIO().hasAlgae() && !intake.getIntakeIO().hasCoral()) {
          currentState = States.INTAKE_CORAL;
        }
      case PREP_TROUGH:
        if (intake.getIntakeIO().hasCoral() && !intake.getIntakeIO().hasAlgae()) {
          currentState = States.PLACE_TROUGH;
          preppedTrough = true;
        }
    }
  }

  public void applyStates() {

    switch (currentState) {
      case STOW:
        currentArmState = ArmStates.STOW;
        currentElevatorState = ElevatorStates.STOW;
        currentWristState = WristStates.STOW;
        currentIntakeState = IntakeStates.STOPPED;
      case INTAKE_CORAL:
        currentArmState = ArmStates.INTAKE_CORAL;
        currentElevatorState = ElevatorStates.INTAKE;
        currentWristState = WristStates.INTAKE;
        currentIntakeState = IntakeStates.STOPPED;
      case PREP_TROUGH:
        currentArmState = ArmStates.TROUGH;
        currentElevatorState = ElevatorStates.TROUGH;
        currentWristState = WristStates.TROUGH;
        currentIntakeState = IntakeStates.STOPPED;
      case PLACE_TROUGH:
        currentIntakeState = IntakeStates.EJECT;
    }

    arm.setDesiredState(currentArmState);
    elevator.setDesiredState(currentElevatorState);
    wrist.setDesiredState(currentWristState);
    intake.setDesiredState(currentIntakeState);
  }

  @Override
  public void periodic() {
    applyStates();
  }
}
