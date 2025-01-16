package frc.robot.utility.localization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The april tags will be used to redefine the global positions of the robot
 * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf#page=37
 * Page 37 talks about the IDs of the april tags
 * If you wish you can define an enumeration for the apriltags IDs that the
 * camera will send to the roboRio
 * for the sake of convienience
 */

public enum AprilTag implements FieldObject {
    // SOUTH is towards the SCORING TABLE
    // EAST is towards the Red Alliance Area
    // WEST is towards the Blue Alliance Area
    // 1 in = 0.0254 m
 
    RED_SOUTH_CORAL_STATION(1, new Translation3d(16.697198, 0.655320, 1.485900), new Rotation2d(2.1991100000000)),
    RED_NORTH_CORAL_STATION(2, new Translation3d(16.697198, 7.396480, 1.485900), new Rotation2d(4.0840700000000)),
    RED_ALGAE_STATION(3, new Translation3d(11.560810, 8.055610, 1.301750), new Rotation2d(4.7123900000000)),
    RED_NORTH_BARGE(4, new Translation3d(9.276080, 6.137656, 1.867916), new Rotation2d(0.0000000000000)),
    RED_SOUTH_BARGE(5, new Translation3d(9.276080, 1.914906, 1.867916), new Rotation2d(0.0000000000000)),
    RED_SOUTH_EAST_CORAL(6, new Translation3d(13.474446, 3.306318, 0.308102), new Rotation2d(5.2359900000000)),
    RED_EAST_CORAL(7, new Translation3d(13.890498, 4.025900, 0.308102), new Rotation2d(0.0000000000000)),
    RED_NORTH_EAST_CORAL(8, new Translation3d(13.474446, 4.745482, 0.308102), new Rotation2d(1.0472000000000)),
    RED_NORTH_WEST_CORAL(9, new Translation3d(12.643358, 4.745482, 0.308102), new Rotation2d(2.0944000000000)),
    RED_WEST_CORAL(10, new Translation3d(12.227306, 4.025900, 0.308102), new Rotation2d(3.1415900000000)),
    RED_SOUTH_WEST_CORAL(11, new Translation3d(12.643358, 3.306318, 0.308102), new Rotation2d(4.1887900000000)),
    BLUE_SOUTH_CORAL_STATION(12, new Translation3d(0.851154, 0.655320, 1.485900), new Rotation2d(0.9424780000000)),
    BLUE_NOTH_CORAL_STATION(13, new Translation3d(0.851154, 7.396480, 1.485900), new Rotation2d(5.3407100000000)),
    BLUE_NORTH_BARGE(14, new Translation3d(8.272272, 6.137656, 1.867916), new Rotation2d(3.1415900000000)),
    BLUE_SOUTH_BARGE(15, new Translation3d(8.272272, 1.914906, 1.867916), new Rotation2d(3.1415900000000)),
    BLUE_ALGAE_STATION(16, new Translation3d(5.987542, -0.003810, 1.301750), new Rotation2d(1.5708000000000)),
    BLUE_SOUTH_WEST_CORAL(17, new Translation3d(4.073906, 3.306318, 0.308102), new Rotation2d(4.1887900000000)),
    BLUE_WEST_CORAL(18, new Translation3d(3.657600, 4.025900, 0.308102), new Rotation2d(3.1415900000000)),
    BLUE_NORTH_WEST_CORAL(19, new Translation3d(4.073906, 4.745482, 0.308102), new Rotation2d(2.0944000000000)),
    BLUE_NORTH_EAST_CORAL(20, new Translation3d(4.904740, 4.745482, 0.308102), new Rotation2d(1.0472000000000)),
    BLUE_EAST_CORAL(21, new Translation3d(5.321046, 4.025900, 0.308102), new Rotation2d(0.0000000000000)),
    BLUE_SOUTH_EAST_CORAL(22, new Translation3d(4.904740, 3.306318, 0.308102), new Rotation2d(5.2359900000000)),
    ;

    private int id;
    private String name;
    private Translation3d position;
    private Rotation2d normalOrientation;

    private AprilTag(int id, Translation3d position, Rotation2d normalOrientation) {
        this.id = id;
        this.position = position;
        this.normalOrientation = normalOrientation;
    }

    public FieldObjectType getType() {
        return FieldObjectType.AprilTag;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return this.name;
    }

    public void setPosition(Translation2d position) {
        this.position = new Translation3d(position.getX(), position.getY(), this.position.getZ());
    }

    public void setPosition3d(Translation3d position) {
        this.position = position;
    }

    @Override
    public Translation2d getPosition() {
        return this.position.toTranslation2d();
    }

    public Translation3d getPosition3d() {
        return this.position;
    }

    public int getID() {
        return this.id;
    }

    public Rotation2d getNormalOrientation() {
        return this.normalOrientation;
    }
}
