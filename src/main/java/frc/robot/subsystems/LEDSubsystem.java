package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Localizers;
import frc.robot.utility.ReefLocations;

public class LEDSubsystem extends SubsystemBase {
    private final static double LOCALIZER_LATENCY_THRESHOLD = 0.1;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(17);
    private static final int DEFAULT_BLUE_R = 0;
    private static final int DEFAULT_BLUE_G = 0;
    private static final int DEFAULT_BLUE_B = 255;
    private static final int WHITE_R = 255;
    private static final int WHITE_G = 255;
    private static final int WHITE_B = 255;
    private static final int ORANGE_R = 255;
    private static final int ORANGE_G = 165;
    private static final int ORANGE_B = 0;
    private static final int GREEN_R = 0;
    private static final int GREEN_G = 255;
    private static final int GREEN_B = 0;
    
    private CoralScorerSubsystem coralScorer;
    private CoralIntakeSubsystem coralIntake;
    private Localizers localizers;

    private boolean aprilTagVisible = false;
    private boolean alignedReefLeft = false;
    private boolean alignedReefRight = false;

    public static final double FINISH_DEADBAND = 0.025; // m
    public static final double FINISH_ANGULAR_DEADBAND = 2 * Unit.DEG; // rad

    public LEDSubsystem(CoralScorerSubsystem coralScorer, CoralIntakeSubsystem coralIntake, Localizers localizers) {
        this.coralScorer = coralScorer;
        this.coralIntake = coralIntake;
        this.localizers = localizers;

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        if (localizers.getVision().getMeasurement().getTimeSince() < LOCALIZER_LATENCY_THRESHOLD) {
            aprilTagVisible = true;
        } else {
            aprilTagVisible = false;
        }

        Pose2d measurement = localizers.getOdometry().getMeasurement().pose;
        Pose2d target = ReefLocations.nearestLeftScoringLocation(measurement);
        alignedReefLeft = (measurement.minus(target).getTranslation().getNorm() <= FINISH_DEADBAND) && (Math.abs(measurement.minus(target).getRotation().getRadians()) <= FINISH_ANGULAR_DEADBAND);

        target = ReefLocations.nearestRightScoringLocation(measurement);
        alignedReefRight = (measurement.minus(target).getTranslation().getNorm() <= FINISH_DEADBAND) && (Math.abs(measurement.minus(target).getRotation().getRadians()) <= FINISH_ANGULAR_DEADBAND);

        if (coralScorer.getSawCoralInLastFrame()) {
            if (alignedReefLeft) {
                fillLeft(GREEN_R, GREEN_G, GREEN_B);
            } else if (alignedReefRight) {
                fillRight(GREEN_R, GREEN_G, GREEN_B);
            } else if (aprilTagVisible) {
                fill(ORANGE_R, ORANGE_G, ORANGE_B);
            } else {
                fill(WHITE_R, WHITE_G, WHITE_B);
            }
        } if (coralIntake.isIntaking()) {
            pulse(WHITE_R, WHITE_G, WHITE_B, 0);
        } else {
            fill(DEFAULT_BLUE_R, DEFAULT_BLUE_G, DEFAULT_BLUE_B);
        }

        update();
    }

    /**
     * Fill LED strip with solid color.
     * @param r
     * @param g
     * @param b
     */
    public void fill(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Fill LED strip with solid color.
     * @param r
     * @param g
     * @param b
     */
    public void fillLeft(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength() / 2; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Fill LED strip with solid color.
     * @param r
     * @param g
     * @param b
     */
    public void fillRight(int r, int g, int b) {
        for (var i = ledBuffer.getLength() / 2; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Blinking animation using a simple sine wave.
     * @param r
     * @param g
     * @param b
     * @param period in seconds
     */
    public void pulse(int r, int g, int b, double period) {
        double time = Timer.getFPGATimestamp();
        double brightness = (Math.sin(((2 * Math.PI) / period) * time) + 1) / 2;

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, (int)(r * brightness), (int)(g * brightness), (int)(b * brightness));
        }
    }

    /**
     * Update LED strip based on buffer.
     */
    private void update() {
        led.setData(ledBuffer);
    }
}
