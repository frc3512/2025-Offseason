package org.frc3512.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc3512.robot.buttons.ControlBoard;
import org.frc3512.robot.buttons.ModeControls;
import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.constants.Constants.GeneralConstants;
import org.frc3512.robot.constants.TunerConstants;
import org.frc3512.robot.subsytems.arm.Arm;
import org.frc3512.robot.subsytems.arm.ArmIOSim;
import org.frc3512.robot.subsytems.drive.Drive;
import org.frc3512.robot.subsytems.drive.GyroIO;
import org.frc3512.robot.subsytems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsytems.drive.ModuleIO;
import org.frc3512.robot.subsytems.drive.ModuleIOSim;
import org.frc3512.robot.subsytems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsytems.elevator.Elevator;
import org.frc3512.robot.subsytems.elevator.ElevatorIOSim;
import org.frc3512.robot.subsytems.intake.Intake;
import org.frc3512.robot.subsytems.intake.IntakeIOSim;
import org.frc3512.robot.subsytems.wrist.Wrist;
import org.frc3512.robot.subsytems.wrist.WristIOSim;
import org.frc3512.robot.superstructure.Superstructure;

@SuppressWarnings("unused")
public class RobotContainer {

  private Arm arm;
  private Elevator elevator;
  private Wrist wrist;

  private Intake intake;

  private Drive drive;

  private Superstructure superstructure = new Superstructure();

  // * Create Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // * Create Buttons
  // For actual mode spedific buttons
  private final ModeControls buttons = ModeControls.getInstance();
  // For all robot modes
  private final ControlBoard controlBoard = ControlBoard.getInstance();

  // ? Create Vision

  public RobotContainer() {

    switch (GeneralConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        Arm.setInstance(new ArmIOSim());
        arm = Arm.getInstance();

        Elevator.setInstance(new ElevatorIOSim());
        elevator = Elevator.getInstance();

        Wrist.setInstance(new WristIOSim());
        wrist = Wrist.getInstance();

        Intake.setInstance(new IntakeIOSim());
        intake = Intake.getInstance();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    configureAxisActions();
  }

  private void configureAxisActions() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controlBoard.getThrottle(),
            () -> controlBoard.getStrafe(),
            () -> controlBoard.getRotation()));
  }

  private void configureButtonBindings() {

    // Reset gyro to 0° when B button is pressed
    controlBoard
        .gyro()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  private void configureBindings() {
    buttons.configureBindings();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
