package org.frc3512.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IButtonControlBoard {

  // * Stow
  Trigger stow();

  Trigger gyro();

  // * Coral
  Trigger intakeCoral();

  Trigger L1();

  Trigger L2();

  Trigger L3();

  Trigger L4();

  Trigger score();

  // * Algae
  Trigger intakeAlgae();

  Trigger deReefA1();

  Trigger deReefA2();

  Trigger barge();

  Trigger process();

  // * Vision
  Trigger allignLeft();

  Trigger allignRight();

  Trigger allignAlgae();

  // * Mode Switching
  Trigger setCoralMode();

  Trigger setAlgaeMode();
}
