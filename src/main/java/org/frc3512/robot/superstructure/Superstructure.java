package org.frc3512.robot.superstructure;

import java.util.ArrayList;
import java.util.List;

import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.constants.Constants;
import org.frc3512.robot.constants.Constants.FieldConstants.ReefSlot;
import org.frc3512.robot.constants.TunerConstants;
import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.arm.ArmStates;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorStates;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeStates;
import org.frc3512.robot.subsystems.wrist.Wrist;
import org.frc3512.robot.subsystems.wrist.WristStates;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Superstructure extends SubsystemBase {

  Arm arm;
  Elevator elevator;
  Wrist wrist;

  Intake intake;

  Drive drive;

  // used for score logic, so we have different scoring but one button
  ElevatorStates scoringLevel = null;

  Boolean coralReady = false;
  Boolean bargeReady = false;
  Boolean processorReady = false;

  @AutoLogOutput(key = "RobotStates/Selected Reef")
  private String selectedReef = "Left"; // Selected Reef Pole

  @AutoLogOutput(key = "RobotStates/Selected Piece")
  private String selectedPiece = "Coral"; // Wanted Visio piece

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // * Create Mode Selector for single driver
  public enum driverMode {
    CORAL,
    ALGAE
  }

  public driverMode currentMode = driverMode.CORAL;

  // All subsystems have a reset to stow if we do not meet conditions for the action
  public Superstructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake, Drive drive) {
    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;

    this.intake = intake;

    this.drive = drive;
  }

  public void configureAxisActions() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * TunerConstants.maxSpeed,
            () -> -controller.getLeftX() * TunerConstants.maxSpeed,
            () -> -controller.getRightX() * TunerConstants.maxAngularRate));
  }

  private void configureButtonBindings() {

    // Reset gyro to 0° when both sticks are pressed
    controller
        .rightStick()
        .and(controller.leftStick())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Mode Switcher
    controller.start().onTrue(Commands.runOnce(() -> setMode(driverMode.CORAL)));
    controller.back().onTrue(Commands.runOnce(() -> setMode(driverMode.ALGAE)));

    // | Intake
    controller
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> doIntakeLogic()))
        .onFalse(Commands.runOnce(() -> doPrepLogic()));

    // | Coral Score
    controller
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> placeLogic(scoringLevel)))
        .onFalse(Commands.runOnce(() -> score()));

    // | Face Buttons
    controller
        .b()
        .onTrue(Commands.runOnce(() -> doPushLogic('b')))
        .onFalse(Commands.runOnce(() -> doReleaseLogic('b')));

    controller
        .a()
        .onTrue(Commands.runOnce(() -> doPushLogic('a')))
        .onFalse(Commands.runOnce(() -> doReleaseLogic('a')));

    controller
        .x()
        .onTrue(Commands.runOnce(() -> doPushLogic('x')))
        .onFalse(Commands.runOnce(() -> doReleaseLogic('x')));

    controller
        .y()
        .onTrue(Commands.runOnce(() -> doPushLogic('y')))
        .onFalse(Commands.runOnce(() -> doReleaseLogic('y')));

    controller.povDown().onTrue(Commands.runOnce(() -> reset()));
  }

  public void configureBindings() {
    configureAxisActions();
    configureButtonBindings();
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
        Commands.runOnce(() -> coralReady = false),
        Commands.runOnce(() -> bargeReady = false),
        Commands.runOnce(() -> processorReady = false),
        Commands.runOnce(() -> scoringLevel = null),
        Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
  }

  // * Coral
  public Command placeLogic(ElevatorStates scoringLevel) {
    return Commands.either(
      placeL4(), 
      placeMid(),
      () -> scoringLevel == ElevatorStates.L4);
  }

  // | L4
  public Command placeL4() {
    return Commands.sequence(Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_L4)));
  }

  // | L2 - L3
  public Command placeMid() {
    return Commands.sequence(Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PLACE_MID)));
  }

  public Command score() {
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
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.INTAKE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.INTAKE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.INTAKE_CORAL)),
        grabCoral());
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
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.INTAKE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.ALGAE_INTAKE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.INTAKE_ALGAE)),
        grabAlgae());
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
        Commands.runOnce(() -> coralReady = true),
        Commands.runOnce(() -> scoringLevel = level));
  }

  public Command prepL4() {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.L4)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.PREP_L4)),
        Commands.waitUntil(() -> elevator.atSetpoint()),
        Commands.waitUntil(() -> arm.atSetpoint()),
        Commands.runOnce(() -> coralReady = true),
        Commands.runOnce(() -> scoringLevel = ElevatorStates.L4));
  }

  public Command prepCoral() {
    if (intake.hasCoral()) {
      return Commands.sequence(
          Commands.runOnce(() -> arm.changeSetpoint(ArmStates.HOLD_CORAL)),
          Commands.runOnce(() -> wrist.changeSetpoint(WristStates.CORAL)));
    } else {
      return reset();
    }
  }

  // | Algae
  public Command prepAlgae() {
    // if (hasAlgae()) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.PREP_ALGAE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.STOW)),
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)),
        Commands.runOnce(() -> intake.changeSetpoint(IntakeStates.HOLD)));
    // } else {
    //     return reset();
    // }
  }

  public Command prepBarge() {
    return Commands.sequence(
        Commands.runOnce(() -> wrist.changeSetpoint(WristStates.ALGAE)),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorStates.BARGE)),
        Commands.runOnce(() -> arm.changeSetpoint(ArmStates.BARGE)),
        Commands.runOnce(() -> bargeReady = true));
  }

  public Command doIntakeLogic() {
    return Commands.either(
      intakeCoral(),
      intakeAlgae(),
      () -> currentMode == driverMode.CORAL);
  }

  public Command doPrepLogic() {
    return Commands.either(
      prepCoral(),
      prepAlgae(),
      () -> currentMode == driverMode.CORAL);
  }

  // Logic for ABXY button PRESS
  public Command doPushLogic(char button) {
    switch (button) {
      case 'b':
      return Commands.either(
        prepTrough(),
        prepProcess(),
        () -> currentMode == driverMode.CORAL);

      case 'a':
      return Commands.either(
        prepMidPlace(ElevatorStates.L2),
        grabAlgaeReef(ElevatorStates.ALGAE_L1),
        () -> currentMode == driverMode.CORAL);

      case 'x':
      return Commands.either(
        prepMidPlace(ElevatorStates.L3),
        prepBarge(),
        () -> currentMode == driverMode.CORAL);

      case 'y':
      return Commands.either(
        prepL4(),
        grabAlgaeReef(ElevatorStates.ALGAE_L2),
        () -> currentMode == driverMode.CORAL);
      
      default:
        return reset();
    }
  }

  // Logic for ABXY button RELEASE
  public Command doReleaseLogic(char button) {
    switch (button) {
      case 'b':
        return Commands.either(
          trough(),
          process(),
          () -> currentMode == driverMode.CORAL);
      
      case 'a':
        if (currentMode == driverMode.ALGAE) {
          return prepAlgae();
        }

      case 'x':
        if (currentMode == driverMode.ALGAE) {
          return scoreBarge();
        }

      default:
        return reset();
    }
  }

  // Gets the closest reef sector to the robot.
  @AutoLogOutput(key = "RobotStates/Nearest Reef")
  public Pose2d getNearestReef() {

    // Grab the alliance color
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Create an array of reef slots based on the alliance color
    ReefSlot[] reefSlots = new ReefSlot[6];

    // Set the reef slots based on the alliance color
    if (alliance == Alliance.Red) {
      reefSlots =
          new ReefSlot[] {
            Constants.FieldConstants.ReefPoses.Reef_1.red,
            Constants.FieldConstants.ReefPoses.Reef_2.red,
            Constants.FieldConstants.ReefPoses.Reef_3.red,
            Constants.FieldConstants.ReefPoses.Reef_4.red,
            Constants.FieldConstants.ReefPoses.Reef_5.red,
            Constants.FieldConstants.ReefPoses.Reef_6.red
          };
    } else {
      reefSlots =
          new ReefSlot[] {
            Constants.FieldConstants.ReefPoses.Reef_1.blue,
            Constants.FieldConstants.ReefPoses.Reef_2.blue,
            Constants.FieldConstants.ReefPoses.Reef_3.blue,
            Constants.FieldConstants.ReefPoses.Reef_4.blue,
            Constants.FieldConstants.ReefPoses.Reef_5.blue,
            Constants.FieldConstants.ReefPoses.Reef_6.blue
          };
    }

    // Create a list of reef middle poses the robot scores offcenter but this is
    // used instead just
    // to select the sector.
    List<Pose2d> reefMiddles = new ArrayList<>();
    for (ReefSlot reefSlot : reefSlots) {
      reefMiddles.add(reefSlot.middle);
    }

    Pose2d currentPos = drive.getPose(); // Get the current robot pose
    Pose2d nearestReefMiddle = currentPos.nearest(reefMiddles); // Get the nearest reef middle pose
    ReefSlot nearestReefSlot =
        reefSlots[
            reefMiddles.indexOf(
                nearestReefMiddle)]; // Get the reef slot of the nearest reef middle pose

    // Return the reef slot based on the selected reef
    if (selectedPiece == "Coral") {
      if (selectedReef == "Left") {
        return nearestReefSlot.left;
      } else if (selectedReef == "Right") {
        return nearestReefSlot.right;
      }
    } else {
      return nearestReefSlot.algae;
    }

    // If the selected reef is invalid return the robot's current pose.
    Logger.recordOutput("Errors", "Invalid Reef Selected '" + selectedReef + "'");
    return nearestReefMiddle;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Data/Driver Mode", currentMode);

    Logger.recordOutput("Data/Coral Ready", coralReady);
    Logger.recordOutput("Data/Barge Ready", bargeReady);
    Logger.recordOutput("Data/Process Ready", processorReady);
  }
}
