package org.frc3512.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {

  private static ControlBoard instance;

  public static ControlBoard getInstance() {
    if (instance == null) {
      instance = new ControlBoard();
    }
    return instance;
  }

  private final IDriveControlBoard driveControlBoard;
  private final IButtonControlBoard buttonControlBoard;

  private ControlBoard() {
    driveControlBoard = GamepadDriveControlBoard.getInstance();
    buttonControlBoard = GamepadButtonControlBoard.getInstance();
  }

  // * Drive
  @Override
  public double getThrottle() {
    return driveControlBoard.getThrottle();
  }

  @Override
  public double getStrafe() {
    return driveControlBoard.getStrafe();
  }

  @Override
  public double getRotation() {
    return driveControlBoard.getRotation();
  }

  @Override
  public Trigger resetGyro() {
    return driveControlBoard.resetGyro();
  }

  // * Button

  // * Stow
  @Override
  public Trigger stow() {
    return buttonControlBoard.stow();
  }

  @Override
  public Trigger gyro() {
    return buttonControlBoard.gyro();
  }

  // * Coral
  @Override
  public Trigger intakeCoral() {
    return buttonControlBoard.intakeCoral();
  }

  @Override
  public Trigger L1() {
    return buttonControlBoard.L1();
  }

  @Override
  public Trigger L2() {
    return buttonControlBoard.L2();
  }

  @Override
  public Trigger L3() {
    return buttonControlBoard.L3();
  }

  @Override
  public Trigger L4() {
    return buttonControlBoard.L4();
  }

  @Override
  public Trigger score() {
    return buttonControlBoard.score();
  }

  // * Algae
  @Override
  public Trigger intakeAlgae() {
    return buttonControlBoard.intakeAlgae();
  }

  @Override
  public Trigger deReefA1() {
    return buttonControlBoard.deReefA1();
  }

  @Override
  public Trigger deReefA2() {
    return buttonControlBoard.deReefA2();
  }

  @Override
  public Trigger barge() {
    return buttonControlBoard.barge();
  }

  @Override
  public Trigger process() {
    return buttonControlBoard.process();
  }

  // * Vision
  @Override
  public Trigger allignLeft() {
    return buttonControlBoard.allignLeft();
  }

  @Override
  public Trigger allignRight() {
    return buttonControlBoard.allignRight();
  }

  @Override
  public Trigger allignAlgae() {
    return buttonControlBoard.allignAlgae();
  }

  // * Mode Switching
  @Override
  public Trigger setCoralMode() {
    return buttonControlBoard.setCoralMode();
  }

  @Override
  public Trigger setAlgaeMode() {
    return buttonControlBoard.setAlgaeMode();
  }
}
