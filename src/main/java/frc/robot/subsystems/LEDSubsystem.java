package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.networking.NetworkingCall;
import frc.robot.utility.networking.NetworkingServer;
import frc.robot.utility.networking.types.NetworkingTag;

public class LEDSubsystem extends SubsystemBase {
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
    
    private final CoralScorerSubsystem coralScorer;
    private boolean aprilTagVisible = false;
    private boolean aprilTagAligned = false;

    public LEDSubsystem(CoralScorerSubsystem coralScorer) {
        this.coralScorer = coralScorer;
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
        fill(DEFAULT_BLUE_R, DEFAULT_BLUE_G, DEFAULT_BLUE_B);
        
        NetworkingServer server = new NetworkingServer();
        server.subscribe("tag", (NetworkingCall<NetworkingTag>) this::handleTagDetection);
    }

    private void handleTagDetection(NetworkingTag tag) {
        if (tag != null) {
            aprilTagVisible = true;
            double centerThreshold = 0.1;
            aprilTagAligned = Math.abs(tag.position.getX()) < centerThreshold;
        } else {
            aprilTagVisible = false;
            aprilTagAligned = false;
        }
    }

    @Override
    public void periodic() {
        if (aprilTagAligned) {
            fill(GREEN_R, GREEN_G, GREEN_B);
        } else if (aprilTagVisible) {
            fill(ORANGE_R, ORANGE_G, ORANGE_B);
        } else if (coralScorer.getSawCoralInLastFrame()) {
            fill(WHITE_R, WHITE_G, WHITE_B);
        } else {
            boolean needsReset = false;
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                if (ledBuffer.getLED(i).red == 0 && 
                    ledBuffer.getLED(i).green == 0 && 
                    ledBuffer.getLED(i).blue == 0) {
                    needsReset = true;
                    break;
                }
            }
            if (needsReset) {
                fill(DEFAULT_BLUE_R, DEFAULT_BLUE_G, DEFAULT_BLUE_B);
            }
        }
    }

    public void fill(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }
}
