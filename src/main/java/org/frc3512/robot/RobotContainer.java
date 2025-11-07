package org.frc3512.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.constants.Constants.GeneralConstants;
import org.frc3512.robot.constants.TunerConstants;
import org.frc3512.robot.subsystems.arm.Arm;
import org.frc3512.robot.subsystems.arm.ArmIO;
import org.frc3512.robot.subsystems.arm.ArmIOSim;
import org.frc3512.robot.subsystems.arm.ArmIOTalonFX;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.drive.GyroIO;
import org.frc3512.robot.subsystems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsystems.drive.ModuleIO;
import org.frc3512.robot.subsystems.drive.ModuleIOSim;
import org.frc3512.robot.subsystems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsystems.elevator.Elevator;
import org.frc3512.robot.subsystems.elevator.ElevatorIO;
import org.frc3512.robot.subsystems.elevator.ElevatorIOSim;
import org.frc3512.robot.subsystems.elevator.ElevatorIOTalonFX;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeIO;
import org.frc3512.robot.subsystems.intake.IntakeIOSim;
import org.frc3512.robot.subsystems.intake.IntakeIOTalonFX;
import org.frc3512.robot.subsystems.wrist.Wrist;
import org.frc3512.robot.subsystems.wrist.WristIO;
import org.frc3512.robot.subsystems.wrist.WristIOSim;
import org.frc3512.robot.subsystems.wrist.WristIOTalonFX;
import org.frc3512.robot.superstructure.Superstructure;
import org.frc3512.robot.superstructure.Superstructure.driverMode;

@SuppressWarnings("unused")
public class RobotContainer {

  private Arm arm;
  private Elevator elevator;
  private Wrist wrist;

  private Intake intake;

  // private Led leds;

  private Drive drive;

  private Superstructure actions;

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // ? Create Vision

  public RobotContainer() {

    switch (GeneralConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        arm = new Arm(new ArmIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());

        intake = new Intake(new IntakeIOTalonFX());

        // leds = new Led(new LedIOReal());

        actions = new Superstructure(arm, elevator, wrist, intake);

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());

        intake = new Intake(new IntakeIOSim());

        actions = new Superstructure(arm, elevator, wrist, intake);

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        break;

      default:
        // Replayed robot, disable IO implementations
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});

        intake = new Intake(new IntakeIO() {});

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    configureBindings();
  }

  private void configureAxisActions() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * TunerConstants.maxSpeed,
            () -> -controller.getLeftX() * TunerConstants.maxSpeed,
            () -> -controller.getRightX() * TunerConstants.maxAngularRate));
  }

  private void configureButtonBindings() {

    // Reset gyro to 0° when B button is pressed
    resetGyro()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Mode Switcher
    switchToCoral().onTrue(Commands.runOnce(() -> actions.setMode(driverMode.CORAL)));
    switchToAlgae().onTrue(Commands.runOnce(() -> actions.setMode(driverMode.ALGAE)));

    // | Intake
    controller
        .rightBumper()
        .onTrue(Commands.runOnce(() -> actions.intakeCoral()))
        .onFalse(Commands.runOnce(() -> actions.prepCoral()));
  }

  private void configureBindings() {
    configureAxisActions();
    configureButtonBindings();
  }

  public Command getAutonomousCommand() {
    return null;
  }

  //  * Define triggers here

  // Gyro
  private Trigger resetGyro() {
    return controller.rightStick().and(controller.leftStick());
  }

  // Superstructure
  private Trigger stow() {
    return controller.povDown();
  }

  // Mode Switcher
  private Trigger switchToCoral() {
    return controller.start();
  }

  private Trigger switchToAlgae() {
    return controller.back();
  }
}
