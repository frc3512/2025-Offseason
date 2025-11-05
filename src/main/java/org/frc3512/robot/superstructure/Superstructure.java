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

  Boolean coralReady = false;
  Boolean bargeReady = false;
  Boolean processorReady = false;

  // * Create Mode Selector for single driver
  public enum driverMode {
    CORAL,
    ALGAE
  }

  public driverMode currentMode;

  // All subsystems have a reset to stow if we do not meet conditions for action
  public Superstructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake) {

    // stateMachine = new StateMachine();

    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;

    this.intake = intake;
  }

  public void setMode(driverMode mode) {
    switch (mode) {
      case CORAL:
        currentMode = driverMode.CORAL;
        break;
      case ALGAE:
        currentMode = driverMode.ALGAE;
        break;
    }
  }

  // * Resst superstructure to stowed position
  public Command stow() {
    return Commands.parallel(
      arm.changeSetpoint(ArmStates.STOW),
      elevator.changeSetpoint(ElevatorStates.STOW),
      wrist.changeSetpoint(WristStates.STOW),
      
      intake.changeSetpoint(IntakeStates.STOPPED));
  }

  // * --- CORAL ---

  // | l1
  public Command prepTrough() {
    if (intake.hasCoral()) {
      return Commands.sequence(

      Commands.parallel(
        arm.changeSetpoint(ArmStates.TROUGH),
        elevator.changeSetpoint(ElevatorStates.TROUGH),
        wrist.changeSetpoint(WristStates.TROUGH),
        
        intake.changeSetpoint(IntakeStates.STOPPED)),

      Commands.runOnce(() -> coralReady = true)

    );

    } else {
      return stow();
    }
  }

  public Command placeTrough() {
    if (coralReady) {
      return Commands.sequence(
        Commands.parallel(
          arm.changeSetpoint(ArmStates.TROUGH),
          elevator.changeSetpoint(ElevatorStates.TROUGH),
          wrist.changeSetpoint(WristStates.TROUGH),
          
          intake.changeSetpoint(IntakeStates.SPIT)),

        Commands.waitSeconds(0.5),

        stow(),

        Commands.runOnce(() -> coralReady = false)
      );
    } else {
      return stow();
    }
  }

  // | L2 / 3
  public Command prepMid() {
    if (intake.hasCoral()) {
      return Commands.sequence(


      );
    } else {
      return stow();
    }
  }

}
