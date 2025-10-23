package org.frc3512.robot.buttons;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ModeControls {

    private static Optional<ModeControls> instance = Optional.empty();

    public enum Mode {
        CORAL,
        ALGAE
    }

    private Mode currentMode = Mode.CORAL;
    private Consumer<Mode> stateChangeConsumer;

    public static ModeControls getInstance() {
        if (instance.isEmpty()) {
            instance = Optional.of(new ModeControls());
        }
        return instance.get();
    }

    public Mode getMode() {
        return currentMode;
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.currentMode != newMode && this.stateChangeConsumer != null) {
            this.stateChangeConsumer.accept(newMode);
        }
    }

    private Trigger modeSpecific(Trigger trigger, Mode mode) {
        return trigger.and(new Trigger(() -> this.currentMode == mode));
    }

    public void configureBindings() {

        ControlBoard.getInstance()
                .setCoralMode()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    maybeTriggerStateChangeConsumer(Mode.CORAL);
                                    setMode(Mode.CORAL);
                                }));

        ControlBoard.getInstance()
                .setAlgaeMode()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    maybeTriggerStateChangeConsumer(Mode.ALGAE);
                                    setMode(Mode.ALGAE);
                                }));

    }

    public Trigger coralMode() {
        return new Trigger(() -> this.currentMode == Mode.CORAL);
    }

    public Trigger algaeMode() {
        return new Trigger(() -> this.currentMode == Mode.ALGAE);
    }

    // Coral Mode
    public Trigger intakeCoral() {
        return modeSpecific(ControlBoard.getInstance().intakeCoral(), Mode.CORAL);
    }

    public Trigger L1() {
        return modeSpecific(ControlBoard.getInstance().L1(), Mode.CORAL);
    }

    public Trigger L2() {
        return modeSpecific(ControlBoard.getInstance().L2(), Mode.CORAL);
    }

    public Trigger L3() {
        return modeSpecific(ControlBoard.getInstance().L3(), Mode.CORAL);
    }

    public Trigger L4() {
        return modeSpecific(ControlBoard.getInstance().L4(), Mode.CORAL);
    }

    public Trigger score() {
        return modeSpecific(ControlBoard.getInstance().score(), Mode.CORAL);
    }

    // Algae Mode
    public Trigger intakeAlgae() {
        return modeSpecific(ControlBoard.getInstance().intakeAlgae(), Mode.ALGAE);
    }

    public Trigger deReefA1() {
        return modeSpecific(ControlBoard.getInstance().deReefA1(), Mode.ALGAE);
    }

    public Trigger deReefA2() {
        return modeSpecific(ControlBoard.getInstance().deReefA2(), Mode.ALGAE);
    }

    public Trigger barge() {
        return modeSpecific(ControlBoard.getInstance().barge(), Mode.ALGAE);
    }

    public Trigger process() {
        return modeSpecific(ControlBoard.getInstance().process(), Mode.ALGAE);
    }

    // Vision
    public Trigger allignLeft() {
        return modeSpecific(ControlBoard.getInstance().allignLeft(), Mode.CORAL);
    }

    public Trigger allignRight() {
        return modeSpecific(ControlBoard.getInstance().allignRight(), Mode.CORAL);
    }

    public Trigger allignAlgae() {
        return modeSpecific(ControlBoard.getInstance().allignAlgae(), Mode.ALGAE);
    }

}
