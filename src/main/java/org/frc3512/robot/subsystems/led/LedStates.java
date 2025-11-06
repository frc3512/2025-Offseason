package org.frc3512.robot.subsystems.led;

import org.frc3512.robot.constants.Constants.LedConstants;

import edu.wpi.first.wpilibj.LEDPattern;

public enum LedStates {

    STOWED(LedConstants.stow),
    
    CORAL(LedConstants.coral),
    ALGAE(LedConstants.algae),

    INTAKE_CORAL(LedConstants.intakeCoral),
    INTAKE_ALGAE(LedConstants.intakeAlgae),

    PREPPING(LedConstants.preping),

    SCORING(LedConstants.scoring);
    
    LEDPattern pattern;

    LedStates(LEDPattern pattern) {
        this.pattern = pattern;
    }

}
