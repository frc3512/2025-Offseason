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
import org.littletonrobotics.junction.Logger;

public class StateMachine extends SubsystemBase {

  private States currentState;

  private ArmStates currentArmState;
  private ElevatorStates currentElevatorState;
  private WristStates currentWristState;
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

      // --- MAIN ---
      case STOW:
        if (!Intake.getInstance().getIntakeIO().hasAlgae() && 
            !Intake.getInstance().getIntakeIO().hasCoral()) {
          currentState = States.STOW;
        }

      // --- CORAL LOGIC ---
      case PREP_CORAL:
        if (!Intake.getInstance().getIntakeIO().hasAlgae() && 
            Intake.getInstance().getIntakeIO().hasCoral()) {
          currentState = States.PREP_CORAL;
        }
      
      case INTAKE_CORAL:
        if (!Intake.getInstance().getIntakeIO().hasAlgae() && 
            !Intake.getInstance().getIntakeIO().hasCoral()) {
          currentState = States.INTAKE_CORAL;
        }
      case PREP_TROUGH:
        if (Intake.getInstance().getIntakeIO().hasCoral() && 
            !Intake.getInstance().getIntakeIO().hasAlgae()) {
          currentState = States.PLACE_TROUGH;
          preppedTrough = true;
        }
        case PREP_L2:
          if (Intake.getInstance().getIntakeIO().hasCoral() && 
              !Intake.getInstance().getIntakeIO().hasAlgae()) {
            currentState = States.PLACE_L2;
            preppedL2 = true;
          }
        case PREP_L3:
          if (Intake.getInstance().getIntakeIO().hasCoral() && 
              !Intake.getInstance().getIntakeIO().hasAlgae()) {
            currentState = States.PLACE_L3;
            preppedL3 = true;
          }
        case PREP_L4:
          if (Intake.getInstance().getIntakeIO().hasCoral() && 
              !Intake.getInstance().getIntakeIO().hasAlgae()) {
            currentState = States.PLACE_L4;
            preppedL4 = true;
          }
        case PLACE_TROUGH:
          if (preppedTrough) {
            if (Intake.getInstance().getIntakeIO().hasCoral() && 
                !Intake.getInstance().getIntakeIO().hasAlgae()) {
              currentState = States.PLACE_TROUGH;
            } else {
              currentState = States.STOW;
              preppedTrough = false;
            }
          }
        case PLACE_L2:
          if (preppedL2) {
            if (Intake.getInstance().getIntakeIO().hasCoral() && 
                !Intake.getInstance().getIntakeIO().hasAlgae()) {
              currentState = States.PLACE_L2;
            } else {
              currentState = States.STOW;
              preppedL2 = false;
            }
          }
        case PLACE_L3:
          if (preppedL3) {
            if (Intake.getInstance().getIntakeIO().hasCoral() && 
                !Intake.getInstance().getIntakeIO().hasAlgae()) {
              currentState = States.PLACE_L3;
            } else {
              currentState = States.STOW;
              preppedL3 = false;
            }
          }
        case PLACE_L4:
          if (preppedL4) {
            if (Intake.getInstance().getIntakeIO().hasCoral() && 
                !Intake.getInstance().getIntakeIO().hasAlgae()) {
              currentState = States.PLACE_L4;
            } else {
              currentState = States.STOW;
              preppedL4 = false;
            }
          }

        // --- ALGAE LOGIC ---
        case INTAKE_ALGAE:
          if (!Intake.getInstance().getIntakeIO().hasAlgae() && 
              !Intake.getInstance().getIntakeIO().hasCoral()) {
            currentState = States.INTAKE_ALGAE;
          }
        case HOLD_ALGAE:
          if (Intake.getInstance().getIntakeIO().hasAlgae() && 
              !Intake.getInstance().getIntakeIO().hasCoral()) {
            currentState = States.HOLD_ALGAE;
          }
        case DE_REEF_A1:
          if (!Intake.getInstance().getIntakeIO().hasAlgae() && 
              !Intake.getInstance().getIntakeIO().hasCoral()) {
            currentState = States.DE_REEF_A1;
          }
        case DE_REEF_A2:
          if (!Intake.getInstance().getIntakeIO().hasAlgae() && 
              
              !Intake.getInstance().getIntakeIO().hasCoral()) {
            currentState = States.DE_REEF_A2;
          }
        case PREP_BARGE:
          if (Intake.getInstance().getIntakeIO().hasAlgae() && 
              !Intake.getInstance().getIntakeIO().hasCoral()) {
            currentState = States.PREP_BARGE;
            preppedBarge = true;
          }
        case PREP_PROCESSOR:
          if (Intake.getInstance().getIntakeIO().hasAlgae() && 
              !Intake.getInstance().getIntakeIO().hasCoral()) {
            currentState = States.PREP_PROCESSOR;
            preppedProcessor = true;
          }
        case EJECT:
          if (preppedBarge) {
            if (Intake.getInstance().getIntakeIO().hasAlgae() && 
                !Intake.getInstance().getIntakeIO().hasCoral()) {
              currentState = States.EJECT;
            } else {
              currentState = States.STOW;
              preppedBarge = false;
            }
          }
          if (preppedProcessor) {
            if (Intake.getInstance().getIntakeIO().hasAlgae() && 
                !Intake.getInstance().getIntakeIO().hasCoral()) {
              currentState = States.EJECT;
            } else {
              currentState = States.STOW;
              preppedProcessor = false;
            }
          }
    }
  }

  public void applyStates() {

    switch (currentState) {
        // --- MAIN ---
      case STOW:
        currentArmState = ArmStates.STOW;
        currentElevatorState = ElevatorStates.STOW;
        currentWristState = WristStates.STOW;
        currentIntakeState = IntakeStates.STOPPED;

        // --- CORAL SETPOINTS ---
      case INTAKE_CORAL:
        currentArmState = ArmStates.INTAKE_CORAL;
        currentElevatorState = ElevatorStates.INTAKE;
        currentWristState = WristStates.INTAKE;
        currentIntakeState = IntakeStates.INTAKE;

      case PREP_CORAL:
        currentArmState = ArmStates.HOLD_CORAL;
        currentElevatorState = ElevatorStates.PREP_CORAL;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.STOPPED;

      case PREP_TROUGH:
        currentArmState = ArmStates.TROUGH;
        currentElevatorState = ElevatorStates.TROUGH;
        currentWristState = WristStates.TROUGH;
        currentIntakeState = IntakeStates.STOPPED;
      case PLACE_TROUGH:
        currentArmState = ArmStates.TROUGH;
        currentElevatorState = ElevatorStates.TROUGH;
        currentWristState = WristStates.TROUGH;
        currentIntakeState = IntakeStates.EJECT;

      case PREP_L2:
        currentArmState = ArmStates.PREP_MID;
        currentElevatorState = ElevatorStates.L3;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.STOPPED;
      case PREP_L3:
        currentArmState = ArmStates.PREP_MID;
        currentElevatorState = ElevatorStates.L3;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.STOPPED;
      case PREP_L4:
        currentArmState = ArmStates.PREP_L4;
        currentElevatorState = ElevatorStates.L4;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.STOPPED;

      case PLACE_L2:
        currentArmState = ArmStates.PLACE_MID;
        currentElevatorState = ElevatorStates.L2;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.PLACE;
      case PLACE_L3:
        currentArmState = ArmStates.PLACE_MID;
        currentElevatorState = ElevatorStates.L3;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.PLACE;
      case PLACE_L4:
        currentArmState = ArmStates.PLACE_L4;
        currentElevatorState = ElevatorStates.L4;
        currentWristState = WristStates.CORAL;
        currentIntakeState = IntakeStates.PLACE;

        // --- ALGAE SETPOINTS ---
      case INTAKE_ALGAE:
        currentArmState = ArmStates.INTAKE_ALGAE;
        currentElevatorState = ElevatorStates.INTAKE;
        currentWristState = WristStates.INTAKE;
        currentIntakeState = IntakeStates.INTAKE;

      case HOLD_ALGAE:
        currentArmState = ArmStates.PREP_ALGAE;
        currentElevatorState = ElevatorStates.INTAKE;
        currentWristState = WristStates.INTAKE;
        currentIntakeState = IntakeStates.HOLD;

      case DE_REEF_A1:
        currentArmState = ArmStates.REMOVE_ALGAE;
        currentElevatorState = ElevatorStates.ALGAE_L1;
        currentWristState = WristStates.ALGAE;
        currentIntakeState = IntakeStates.INTAKE;
      case DE_REEF_A2:
        currentArmState = ArmStates.REMOVE_ALGAE;
        currentElevatorState = ElevatorStates.ALGAE_L2;
        currentWristState = WristStates.ALGAE;
        currentIntakeState = IntakeStates.INTAKE;

      case PREP_BARGE:
        currentArmState = ArmStates.BARGE;
        currentElevatorState = ElevatorStates.BARGE;
        currentWristState = WristStates.ALGAE;
        currentIntakeState = IntakeStates.HOLD;
      case PREP_PROCESSOR:
        currentArmState = ArmStates.PROCESS;
        currentElevatorState = ElevatorStates.PROCESSOR;
        currentWristState = WristStates.ALGAE;
        currentIntakeState = IntakeStates.HOLD;

      // Leave subsystem states as they are to allow for ejection at either barge or processor
      case EJECT:
        currentIntakeState = IntakeStates.EJECT;
    }

    Arm.getInstance().setDesiredState(currentArmState);
    Elevator.getInstance().setDesiredState(currentElevatorState);
    Wrist.getInstance().setDesiredState(currentWristState);
    Intake.getInstance().setDesiredState(currentIntakeState);
  }

  @Override
  public void periodic() {
    applyStates();

    Logger.recordOutput("Arm State", currentArmState.state);
    Logger.recordOutput("Elevator State", currentElevatorState.state);
    Logger.recordOutput("Wrist State", currentWristState.state);

    Logger.recordOutput("Intake State", currentIntakeState.speed);
  }
}
