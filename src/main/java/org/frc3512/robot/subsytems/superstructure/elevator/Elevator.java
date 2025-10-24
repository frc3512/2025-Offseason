package org.frc3512.robot.subsytems.superstructure.elevator;

import org.frc3512.robot.subsytems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ElevatorStates desiredState = ElevatorStates.STOW;

    public static Elevator instance;

    public static Elevator setInstance(ElevatorIO io) {
        instance = new Elevator(io);
        return instance;
    }

    public static Elevator getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Elevator subsystem not initialized yet.");
        }
        return instance;
    }

    public void setDesiredState(ElevatorStates target) {
        this.desiredState = target;
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public ElevatorIO getElevatorIO() {
        return io;
    }

    @Override
    public void periodic() {

        applyStates();

        io.updateInputes(inputs);

        Logger.processInputs("Elevator", inputs);

    }

    public void applyStates() {
        io.setDesiredState(desiredState);
    }
    
}
