package org.frc3512.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.arm.ArmStates;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeStates;
import org.frc3512.robot.subsystems.wrist.Wrist;
import org.frc3512.robot.subsystems.wrist.WristStates;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

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

  public driverMode currentMode = driverMode.CORAL;

  // All subsystems have a reset to stow if we do not meet conditions for the action
  public Superstructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake) {
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

  // * Reset / Defualt
  public Command reset() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.STOPPED)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.STOW)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.STOW)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
        Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
  }

  // * Coral

  // | L4
  public Command placeL4() {
    return Commands.sequence(Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_L4)));
  }

  public Command scoreL4() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.PLACE)),
        Commands.waitSeconds(0.5),
        reset());
  }

  // | L2 - L3
  public Command placeMid() {
    return Commands.sequence(Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_MID)));
  }

  public Command scoreMid() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.PLACE)),
        Commands.waitSeconds(0.5),
        reset());
  }

  // | L1
  public Command prepTrough() {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.TROUGH)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.TROUGH)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.TROUGH)),
        Commands.runOnce(() -> coralReady = true));
  }

  public Command trough() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.SPIT)),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.STOPPED)),
        reset());
  }

  // | Intake
  public Command intakeCoral() {
    // if (!hasCoral()) {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.INTAKE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.INTAKE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.INTAKE_CORAL)),
        grabCoral());
    // } else {
    //     return reset();
    // }
  }

  public Command grabCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.INTAKE)),
        Commands.waitUntil(() -> intake.hasCoral()),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.STOPPED)));
  }

  // * Algae

  // | De-Reef
  public Command grabAlgaeReef(ElevatorStates level) {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)),
        Commands.runOnce(() -> elevator.changeSetpoint(level)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.REMOVE_ALGAE)),
        grabAlgae());
  }

  // | Process
  public Command prepProcess() {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.STOW)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PROCESS)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.PROCESS)),
        Commands.runOnce(() -> processorReady = true));
  }

  public Command process() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.EJECT)),
        Commands.waitSeconds(1),
        reset());
  }

  // | Barge
  public Command scoreBarge() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.EJECT)),
        Commands.waitSeconds(0.5),
        reset());
  }

  // | Intake
  public Command intakeAlgae() {
    // if (getPiece() == '!') {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.INTAKE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.ALGAE_INTAKE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.INTAKE_ALGAE)),
        grabAlgae());
    // } else {
    //     return reset();
    // }
  }

  public Command grabAlgae() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.INTAKE)),
        Commands.waitUntil(() -> intake.hasAlgae()),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.STOPPED)));
  }

  // * Prep

  // | Coral

  // | Used for L2 & l3
  public Command prepMidPlace(ElevatorStates level) {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
        Commands.runOnce(() -> elevator.changeSetpoint(level)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.HOLD_CORAL)),
        Commands.waitUntil(() -> elevator.atSetpoint()),
        Commands.waitUntil(() -> arm.atSetpoint()),
        Commands.runOnce(() -> coralReady = true));
  }

  public Command prepL4() {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.L4)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PREP_L4)),
        Commands.waitUntil(() -> elevator.atSetpoint()),
        Commands.waitUntil(() -> arm.atSetpoint()),
        Commands.runOnce(() -> coralReady = true));
  }

  public Command prepCoral() {
    // if (intake.hasCoral()) {
    return Commands.sequence(
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.HOLD_CORAL)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)));
    // } else {
    //     return reset();
    // }
  }

  // | Algae
  public Command prepAlgae() {
    // if (hasAlgae()) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_ALGAE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.STOW)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)));
    // } else {
    //     return reset();
    // }
  }

  public Command prepBarge() {
    // if (getPiece() == 'A') {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.BARGE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.BARGE)),
        Commands.runOnce(() -> bargeReady = true));
    // } else {
    //     return reset();
    // }
  }

  public void doIntakeLogic() {
    if (currentMode == driverMode.CORAL) {
      intakeCoral();
    } else {
      intakeAlgae();
    }
  }

  public void doPrepLogic() {
    if (currentMode == driverMode.CORAL) {
      prepCoral();
    } else {
      prepAlgae();
    }
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Data/Driver Mode", currentMode);

    Logger.recordOutput("Data/Coral Ready", coralReady);
    Logger.recordOutput("Data/Barge Ready", bargeReady);
    Logger.recordOutput("Data/Process Ready", processorReady);
  }
}
