package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.TitanProcessingLocalizer;

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
    private TitanProcessingLocalizer visionLocalizer;

    private boolean aprilTagVisible = false;
    private boolean aprilTagAligned = false;

    public LEDSubsystem(CoralScorerSubsystem coralScorer, CoralIntakeSubsystem coralIntake, TitanProcessingLocalizer visionLocalizer) {
        this.coralScorer = coralScorer;
        this.coralIntake = coralIntake;
        this.visionLocalizer = visionLocalizer;

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        if (visionLocalizer.getMeasurement().getLatency() < LOCALIZER_LATENCY_THRESHOLD) {
            aprilTagVisible = true;
        } else {
            aprilTagVisible = false;
        }

        for(int i = )

        if (coralScorer.getSawCoralInLastFrame()) {
            if (aprilTagAligned) {
                fill(GREEN_R, GREEN_G, GREEN_B);
            } else if (aprilTagVisible) {
                fill(ORANGE_R, ORANGE_G, ORANGE_B);
            } else {
                fill(WHITE_R, WHITE_G, WHITE_B);
            }
        } else if (coralIntake.isIntaking()) {
            pulse(0, 0, 0, 0);
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
