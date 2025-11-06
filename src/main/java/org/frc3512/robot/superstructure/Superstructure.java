package org.frc3512.robot.superstructure;

import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.arm.ArmStates;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeStates;
import org.frc3512.robot.subsystems.led.Led;
import org.frc3512.robot.subsystems.wrist.Wrist;
import org.frc3512.robot.subsystems.wrist.WristStates;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  // private StateMachine stateMachine;

  Arm arm;
  Elevator elevator;
  Wrist wrist;

  Intake intake;

  Led leds;

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
  public Superstructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake, Led leds) {
    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;

    this.intake = intake;

    this.leds = leds;
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
    return Commands.sequence(
        Commands.runOnce(
            () ->
                setState(
                    ArmStates.STOW, ElevatorStates.STOW, WristStates.STOW, IntakeStates.STOPPED)),
        Commands.runOnce(() -> coralReady = false),
        Commands.runOnce(() -> bargeReady = false),
        Commands.runOnce(() -> processorReady = false));
  }

  // * --- CORAL ---

  // | Intake
  public Command intakeCoral() {
    if (!intake.hasCoral()) {
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  setState(
                      ArmStates.INTAKE_CORAL,
                      ElevatorStates.INTAKE,
                      WristStates.INTAKE,
                      IntakeStates.INTAKE)));

    } else {
      return stow();
    }
  }

  public Command prepCoral() {
    if (intake.hasCoral()) {
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  setState(
                      ArmStates.STOW,
                      ElevatorStates.PREP_CORAL,
                      WristStates.CORAL,
                      IntakeStates.STOPPED)));
    } else {
      return stow();
    }
  }

  // | l1
  public Command prepTrough() {
    if (intake.hasCoral()) {
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  setState(
                      ArmStates.TROUGH,
                      ElevatorStates.TROUGH,
                      WristStates.TROUGH,
                      IntakeStates.STOPPED)),
          Commands.runOnce(() -> coralReady = true));

    } else {
      return stow();
    }
  }

  public Command placeTrough() {
    if (coralReady) {
      return Commands.sequence(
          Commands.runOnce(() -> setState(null, null, null, IntakeStates.SPIT)),
          Commands.waitSeconds(0.5),
          stow(),
          Commands.runOnce(() -> coralReady = false));
    } else {
      return stow();
    }
  }

  // | L2 / 3
  public Command prepMid(ElevatorStates level) {
    if (intake.hasCoral()) {
      return Commands.sequence(
          Commands.parallel(
              elevator.changeSetpoint(level),
              arm.changeSetpoint(ArmStates.PREP_MID),
              wrist.changeSetpoint(WristStates.CORAL),
              intake.changeSetpoint(IntakeStates.STOPPED)),
          Commands.runOnce(() -> coralReady = true));

    } else {
      return stow();
    }
  }

  // * --- Double Binding logic
  public Command doIntakeLogic() {
    if (currentMode == driverMode.CORAL) {
      return intakeCoral();
    } else {
      // intakeAlgae();
      return null;
    }
  }

  public Command doPrepLogic() {
    if (currentMode == driverMode.CORAL) {
      return prepCoral();
    } else {
      // return prepAlgae();
      return null;
    }
  }

  // * --- Full system updates
  // ! null parameters will result in leaving the current state as is
  public Command setState(
      ArmStates armState,
      ElevatorStates elevatorState,
      WristStates wristState,
      IntakeStates intakeState) {

    return Commands.parallel(
        arm.changeSetpoint(armState),
        elevator.changeSetpoint(elevatorState),
        wrist.changeSetpoint(wristState),
        intake.changeSetpoint(intakeState));
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Data/Driver Mode", currentMode);

    Logger.recordOutput("Data/Coral Ready", coralReady);
    Logger.recordOutput("Data/Barge Ready", bargeReady);
    Logger.recordOutput("Data/Process Ready", processorReady);
  }
}
