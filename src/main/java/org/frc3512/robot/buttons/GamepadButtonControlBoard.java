package org.frc3512.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadButtonControlBoard implements IButtonControlBoard {

    private static GamepadButtonControlBoard instance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadButtonControlBoard();
        }
        return instance;
    }

    private final CommandXboxController controller;

    private GamepadButtonControlBoard() {
        controller = new CommandXboxController(0);
    }
    
    // * Stow
    @Override
    public Trigger reset() {
        return controller.povDown();
    }
    
    // * Coral
    @Override
    public Trigger intakeCoral() {
        return controller.rightTrigger();
    }
    @Override
    public Trigger L1() {
        return controller.b();
    }

    @Override
    public Trigger L2() {
        return controller.a();
    }

    @Override
    public Trigger L3() {
        return controller.x();
    }

    @Override
    public Trigger L4() {
        return controller.y();
    }

    @Override
    public Trigger score() {
        return controller.leftTrigger();
    }

    // * Algae
    @Override
    public Trigger intakeAlgae() {
        return controller.rightBumper();
    }

    @Override
    public Trigger deReefA1() {
        return controller.a();
    }

    @Override
    public Trigger deReefA2() {
        return controller.y();
    }

    @Override
    public Trigger barge() {
        return controller.x();
    }

    @Override
    public Trigger process() {
        return controller.b();
    }

    // * Vision
    @Override
    public Trigger allignLeft() {
        return controller.leftBumper();
    }

    @Override
    public Trigger allignRight() {
        return controller.rightBumper();
    }

    @Override
    public Trigger allignAlgae() {
        return controller.rightBumper().and(controller.leftBumper());
    }

    // * Mode Switching
    @Override
    public Trigger setCoralMode() {
        return controller.start();
    }

    @Override
    public Trigger setAlgaeMode() {
        return controller.back();
    }
    
}
