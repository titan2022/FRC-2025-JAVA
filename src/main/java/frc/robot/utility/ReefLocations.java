// SPDX-License-Identifier: LicenseRef-WPILib-License
// SPDX-FileCopyrightText: 
// Based on https://raw.githubusercontent.com/icrobotics-team167/2025_Reefscape/ec8a829dcfff950767fefff34e5efa9f940eec9f/src/main/java/frc/cotc/util/ReefLocations.java

package frc.robot.utility;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class ReefLocations {
  private ReefLocations() {}

  public static final Pose2d[] BLUE_POSES;
  public static final Pose2d[] RED_POSES;

  public static final Pose2d[] BLUE_L1_POSES;
  public static final Pose2d[] RED_L1_POSES;

  public static final Translation2d BLUE_REEF;
  public static final Translation2d RED_REEF;

  public static final double OFFSET_FROM_REEF_CENTER = .173;
  public static final double L1_OFFSET_FROM_REEF_CENTER = .55;

  static StructArrayPublisher<Pose2d> bluePosePub = NetworkTableInstance.getDefault().getStructArrayTopic("blue reef poses", Pose2d.struct).publish();
  static StructArrayPublisher<Pose2d> redPosePub = NetworkTableInstance.getDefault().getStructArrayTopic("red reef poses", Pose2d.struct).publish();

  static {
    //noinspection OptionalGetWithoutIsPresent
    double tag18X = Constants.tagLayout.getTagPose(18).get().getX();
    //noinspection OptionalGetWithoutIsPresent
    double tag21X = Constants.tagLayout.getTagPose(21).get().getX();
    BLUE_REEF = new Translation2d((tag18X + tag21X) / 2, Constants.FIELD_WIDTH_METERS / 2);

    var A =
        new Pose2d(
            tag18X - Constants.RobotSize.LONG_RADIUS,
            Constants.FIELD_WIDTH_METERS / 2 + OFFSET_FROM_REEF_CENTER,
            Rotation2d.kZero);
    var B =
        new Pose2d(
            tag18X - Constants.RobotSize.LONG_RADIUS,
            Constants.FIELD_WIDTH_METERS / 2 - OFFSET_FROM_REEF_CENTER,
            Rotation2d.kZero);

    BLUE_POSES = new Pose2d[12];
    BLUE_POSES[0] = A;
    BLUE_POSES[1] = B;
    for (int i = 2; i < 12; i += 2) {
      var rotAngle = Rotation2d.fromDegrees(30 * i);
      BLUE_POSES[i] = A.rotateAround(BLUE_REEF, rotAngle);
      BLUE_POSES[i + 1] = B.rotateAround(BLUE_REEF, rotAngle);
    }

    RED_REEF = BLUE_REEF.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    RED_POSES = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      RED_POSES[i] = BLUE_POSES[i].rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    }

    //For L1 Poses
    var C =
        new Pose2d(
            tag18X - Constants.RobotSize.LONG_RADIUS,
            Constants.FIELD_WIDTH_METERS / 2 + L1_OFFSET_FROM_REEF_CENTER,
            Rotation2d.kZero);
    var D =
        new Pose2d(
            tag18X - Constants.RobotSize.LONG_RADIUS,
            Constants.FIELD_WIDTH_METERS / 2 - L1_OFFSET_FROM_REEF_CENTER,
            Rotation2d.kZero);

    BLUE_L1_POSES = new Pose2d[12];
    BLUE_L1_POSES[0] = C;
    BLUE_L1_POSES[1] = D;
    for (int i = 2; i < 12; i += 2) {
      var rotAngle = Rotation2d.fromDegrees(30 * i);
      BLUE_L1_POSES[i] = C.rotateAround(BLUE_REEF, rotAngle);
      BLUE_L1_POSES[i + 1] = D.rotateAround(BLUE_REEF, rotAngle);
    }

    RED_L1_POSES = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      RED_L1_POSES[i] = BLUE_L1_POSES[i].rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    }

    bluePosePub.set(BLUE_POSES);
    redPosePub.set(RED_POSES);
  }

  public enum ReefBranch {
    A(0),
    B(1),
    C(2),
    D(3),
    E(4),
    F(5),
    G(6),
    H(7),
    I(8),
    J(9),
    K(10),
    L(11);

    final int id;

    ReefBranch(int id) {
      this.id = id;
    }
  }

  public static Pose2d getScoringLocation(ReefBranch reefBranch) {
    return (Constants.getColor() == Alliance.Red ? RED_POSES : BLUE_POSES)[reefBranch.id];
  }

  public static Pose2d nearestLeftScoringLocation(Pose2d currentLocation) {
    Pose2d[] poses = Constants.getColor() == Alliance.Red ? RED_POSES : BLUE_POSES;

    double minDistance = Double.POSITIVE_INFINITY;
    Pose2d minPose = poses[0];
    for(int i = 0; i < 12; i+=2) {
      if(poses[i].minus(currentLocation).getTranslation().getNorm() < minDistance) {
        minDistance = poses[i].minus(currentLocation).getTranslation().getNorm();
        minPose = poses[i];
      }
    }

    return minPose;
  }

  public static Pose2d nearestRightScoringLocation(Pose2d currentLocation) {
    Pose2d[] poses = Constants.getColor() == Alliance.Red ? RED_POSES : BLUE_POSES;

    double minDistance = Double.POSITIVE_INFINITY;
    Pose2d minPose = poses[0];
    for(int i = 1; i < 12; i+=2) {
      if(poses[i].minus(currentLocation).getTranslation().getNorm() < minDistance) {
        minDistance = poses[i].minus(currentLocation).getTranslation().getNorm();
        minPose = poses[i];
      }
    }

    return minPose;
  }

  public static Pose2d nearestLeftL1ScoringLocation(Pose2d currentLocation) {
    Pose2d[] poses = Constants.getColor() == Alliance.Red ? RED_L1_POSES : BLUE_L1_POSES;

    double minDistance = Double.POSITIVE_INFINITY;
    Pose2d minPose = poses[0];
    for(int i = 0; i < 12; i+=2) {
      if(poses[i].minus(currentLocation).getTranslation().getNorm() < minDistance) {
        minDistance = poses[i].minus(currentLocation).getTranslation().getNorm();
        minPose = poses[i];
      }
    }

    return minPose;
  }

  public static Pose2d nearestRightL1ScoringLocation(Pose2d currentLocation) {
    Pose2d[] poses = Constants.getColor() == Alliance.Red ? RED_L1_POSES : BLUE_L1_POSES;

    double minDistance = Double.POSITIVE_INFINITY;
    Pose2d minPose = poses[0];
    for(int i = 1; i < 12; i+=2) {
      if(poses[i].minus(currentLocation).getTranslation().getNorm() < minDistance) {
        minDistance = poses[i].minus(currentLocation).getTranslation().getNorm();
        minPose = poses[i];
      }
    }

    return minPose;
  }
}
