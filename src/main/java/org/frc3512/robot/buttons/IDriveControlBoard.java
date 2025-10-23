package org.frc3512.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    Trigger resetGyro();
}