package org.frc3512.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {
  public static class GeneralConstants {

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
  }

  public static class ArmConstants {
    public static final int ID = 15;

    public static final double kP = 30;
    public static final double kI = 0.0;
    public static final double kD = 0;
    public static final double kA = 25;

    public static final double GEAR_RATIO = 23.0 * (45.0 / 12.0);
    public static final double TOLERANCE = 5; // 5 degress of tolerance

    public static final DCMotor simMotor = DCMotor.getFalcon500(1);

    public static final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
            .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKA(kA));
  }

  public static class ElevatorConstants {
    public static final int frontMotorID = 13;
    public static final int backMotorID = 14;

    public static final double kP = 1.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kG = 0.4;

    public static final double PULLEY_CIRCUMFERENCE = 1.8798 * Math.PI;
    public static final double GEAR_RATIO = 50.0 / 11.0;

    public static final DCMotor simMotor = DCMotor.getFalcon500(2);

    public static final double minHeight = 0.0; // Inches
    public static final double maxHeight = 56.0; // Inches
    public static final double heightTolerance = 1.0; // Inches

    public static final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
            .withSlot0(
                new Slot0Configs()
                    .withKP(kP)
                    .withKG(kG)
                    .withGravityType(GravityTypeValue.Elevator_Static));
  }

  public static class IntakeConstants {

    public static final int motorID = 17;
    public static final int sensorID = 31;

    public static final double GEAR_RATIO = 1.0;

    public static final DCMotor simMotor = DCMotor.getFalcon500(1);

    public static final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO));

    public static final CanandcolorSettings sensorConfig =
        new CanandcolorSettings()
            .setColorFramePeriod(0.040)
            .setLampLEDBrightness(0.35) // 35% brightness
            .setAlignColorFramesToIntegrationPeriod(true)
            .setProximityIntegrationPeriod(ProximityPeriod.k20ms);
  }

  public static class LedConstants {
    public static Distance ledSpacing = Meters.of(1.0 / 35.0);

    public static LEDPattern stow = LEDPattern.solid(Color.kRed);

    public static LEDPattern coral = LEDPattern.solid(Color.kWhite);
    public static LEDPattern algae = LEDPattern.solid(Color.kCyan);

    public static LEDPattern prep = LEDPattern.solid(Color.kOrange);
    public static LEDPattern score = LEDPattern.rainbow(255, 128);

    public static LEDPattern intakeCoral = coral.blink(Seconds.of(1));
    public static LEDPattern intakeAlgae = algae.blink(Seconds.of(1));

    public static LEDPattern preping = prep.blink(Seconds.of(3), Seconds.of(1.5));
    public static LEDPattern scoring =
        score.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), ledSpacing);
  }

  public static class WristConstants {
    public static final double kP = 35;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double GEAR_RATIO = 10.178;

    public static final int ID = 16;

    public static final DCMotor simMotor = DCMotor.getFalcon500(1);

    public static final double TOLERANCE =
        5; // 5 degrees of tolerance to allow contact with hardstops while not being to restrictive

    public static final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
            .withSlot0(new Slot0Configs().withKP(kP));
  }

  public static class VisionConstants {
    public static final String leftCam = "LeftCam"; // Previously "ElevatorCam"
    public static final String rightCam = "RightCam"; // Previously "ClimberCam"

    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Transform3d leftCameraOffset =
        new Transform3d( // Always measure from center of the robot
            Units.inchesToMeters(6.0), // Forward / Back
            Units.inchesToMeters(11), // Left / Right
            Units.inchesToMeters(10.5), // Height
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0)));

    public static final Transform3d rightCameraOffset =
        new Transform3d( // Always measure from center of the robot
            Units.inchesToMeters(6.0), // Forward / Back
            Units.inchesToMeters(-11), // Left / Right
            Units.inchesToMeters(10.5), // Height
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0)));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(1, 1, 2);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
  }

  public static class AutoConstants {
    public static final double xP = 10;
    public static final double xI = 0.0;
    public static final double xD = 0.0;

    public static final double yP = 10;
    public static final double yI = 0.0;
    public static final double yD = 0.0;

    public static final double thetaP = 7.5;
    public static final double thetaI = 0.0;
    public static final double thetaD = 0.0;
  }

  public static class AimingConstants {
    public static final double xP = 10;
    public static final double xI = 0.0;
    public static final double xD = 0.0;

    public static final double yP = 10;
    public static final double yI = 0.0;
    public static final double yD = 0.0;

    public static final double thetaP = 7.5;
    public static final double thetaI = 0.0;
    public static final double thetaD = 0.0;

    public static final TrapezoidProfile.Constraints aimingTranslationConstraints =
        new TrapezoidProfile.Constraints(1, 2);
    public static final TrapezoidProfile.Constraints aimingRotationConstraints =
        new TrapezoidProfile.Constraints(Units.rotationsToRadians(1), Units.rotationsToRadians(2));
  }

  // Credit to 6657
  public static class FieldConstants {
    private static Pose2d getRedReefPose(Pose2d reefPose) {
      return new Pose2d(
          reefPose.getTranslation().getX() + 8.565,
          reefPose.getTranslation().getY(),
          reefPose.getRotation());
    }

    public static class ReefSlot {
      public Pose2d middle;
      public Pose2d left;
      public Pose2d right;
      public Pose2d algae;

      ReefSlot(Pose2d middle, Pose2d left, Pose2d right, Pose2d algae) {
        this.middle = middle;
        this.left = left;
        this.right = right;
        this.algae = algae;
      }
    }

    public static enum ReefPoses {
      Reef_1(new Pose2d(3.17, 4.01, Rotation2d.fromDegrees(0))),
      Reef_2(new Pose2d(3.78, 5.09, Rotation2d.fromDegrees(-60))),
      Reef_3(new Pose2d(5.16, 5.16, Rotation2d.fromDegrees(-120))),
      Reef_4(new Pose2d(5.6, 4.03, Rotation2d.fromDegrees(-180))),
      Reef_5(new Pose2d(5.11, 2.91, Rotation2d.fromDegrees(120))),
      Reef_6(new Pose2d(3.84, 2.91, Rotation2d.fromDegrees(60)));

      public ReefSlot blue;
      public ReefSlot red;

      // Shift the pose to the robot's left
      public Pose2d getLeftPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(0.0, -0.16, new Rotation2d()));
      }

      // Shift the pose to the robot's right
      public Pose2d getRightPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.18, -0.16, new Rotation2d()));
      }

      // Shift the pose to the center
      public Pose2d getAlgaePose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.09, -0.1, new Rotation2d()));
      }

      ReefPoses(Pose2d pose) {
        this.blue = new ReefSlot(pose, getLeftPose(pose), getRightPose(pose), getAlgaePose(pose));
        this.red =
            new ReefSlot(
                getRedReefPose(pose),
                getRedReefPose(getLeftPose(pose)),
                getRedReefPose(getRightPose(pose)),
                getRedReefPose(getAlgaePose(pose)));
      }
    }
  }
}
