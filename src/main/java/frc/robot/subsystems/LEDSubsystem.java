package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Localizers;
import frc.robot.utility.ReefLocations;

public class LEDSubsystem extends SubsystemBase {
    private final static double LOCALIZER_LATENCY_THRESHOLD = 0.1;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(50);
    
    private AddressableLEDBufferView ledBufferViewLeft = new AddressableLEDBufferView(ledBuffer, 0, 24); 
    private AddressableLEDBufferView ledBufferViewRight = new AddressableLEDBufferView(ledBuffer, 25, 49).reversed(); 

    private final double BRIGHTNESS = 0.5;

    private CoralScorerSubsystem coralScorer;
    private CoralIntakeSubsystem coralIntake;
    private AlgaeIntakeSubsystem algaeIntakeSubsystem;
    private Localizers localizers;

    private boolean aprilTagVisible = false;
    private boolean alignedReefLeft = false;
    private boolean alignedReefRight = false;

    public static final double FINISH_DEADBAND = 0.025; // m
    public static final double FINISH_ANGULAR_DEADBAND = 2 * Unit.DEG; // rad

    LEDPattern blue = LEDPattern.solid(new Color(0*BRIGHTNESS,0*BRIGHTNESS,255*BRIGHTNESS));
    LEDPattern white = LEDPattern.solid(new Color(255*BRIGHTNESS,255*BRIGHTNESS,255*BRIGHTNESS));
    LEDPattern orange = LEDPattern.solid(new Color(255*BRIGHTNESS,50*BRIGHTNESS,0*BRIGHTNESS));
    LEDPattern purple = LEDPattern.solid(new Color(123*BRIGHTNESS,17*BRIGHTNESS,245*BRIGHTNESS));

    LEDPattern rainbowPattern = LEDPattern.rainbow(255, 255);
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern scrollingRainbowPattern = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), ledSpacing);

    LEDPattern greenTeal = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(0*BRIGHTNESS,100*BRIGHTNESS,0*BRIGHTNESS), new Color(15 * BRIGHTNESS, 50 * BRIGHTNESS, 50 * BRIGHTNESS)); 
    LEDPattern greenTealScroll = greenTeal.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), ledSpacing);
    LEDPattern breathe = white.breathe(Seconds.of(0.5));

    public LEDSubsystem(CoralScorerSubsystem coralScorer, CoralIntakeSubsystem coralIntake, AlgaeIntakeSubsystem algaeIntakeSubsystem, Localizers localizers) {
        this.coralScorer = coralScorer;
        this.coralIntake = coralIntake;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
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

        if (coralScorer.canSeeCoral()) {
            if (alignedReefLeft) {
                purple.applyTo(ledBufferViewRight);
            } else if (alignedReefRight) {
                purple.applyTo(ledBufferViewLeft);

            } else if (aprilTagVisible) {
                orange.applyTo(ledBuffer);
            } else {
                blue.applyTo(ledBuffer);
            }
        } else if (coralIntake.isIntaking()) {
            breathe.applyTo(ledBuffer);
        } else if (algaeIntakeSubsystem.getHasAlgae()){
            greenTealScroll.applyTo(ledBufferViewLeft);
            greenTealScroll.applyTo(ledBufferViewRight);
        } else {
            // fill(DEFAULT_BLUE_R, DEFAULT_BLUE_G, DEFAULT_BLUE_B);
            scrollingRainbowPattern.applyTo(ledBufferViewLeft);
            scrollingRainbowPattern.applyTo(ledBufferViewRight);
        }

        update();
    }

    /**
     * Update LED strip based on buffer.
     */
    private void update() {
        led.setData(ledBuffer);

    }
}
