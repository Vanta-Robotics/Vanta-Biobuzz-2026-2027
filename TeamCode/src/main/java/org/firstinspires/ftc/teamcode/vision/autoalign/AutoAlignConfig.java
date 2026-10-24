package org.firstinspires.ftc.teamcode.vision.autoalign;

import org.firstinspires.ftc.teamcode.vision.autoalign.AlignGeometry.Vector;

/** The only file to edit for this feature's physical measurements and tuning. */
public final class AutoAlignConfig {
    private AutoAlignConfig() { }
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int PIPELINE = 0;

    // CAD mounting geometry: docs/limelight-cad-measurements.md (2026-10-24).
    // Team confirmed the physical mount matches CAD and shots travel toward the Limelight.
    // The exported camera is its front enclosure only. This position is the aperture
    // center, a provisional optical-origin reference; validate with live target poses.
    public static final boolean CAMERA_MOUNT_VERIFIED = false;
    // Aiming axes HERE: right, forward toward the shooter/camera (+CAD X), up.
    // Origin: midpoint of the four wheel centers, projected onto the nominal floor.
    // These aiming axes do not redefine the wheel branch's driving/IMU axes.
    public static final Vector CAMERA_POSITION = new Vector(-0.0031791, 0.1552402, 0.2642572);
    // CAD has two launch lanes, 135.196 mm apart. A wheel center is NOT a ball
    // release point; select the active lane and establish its exit before filling this.
    public static final Vector SHOOTER_EXIT_POSITION = new Vector(Double.NaN, Double.NaN, Double.NaN);
    public static final double CAMERA_ROLL_CLOCKWISE_DEG = 0;
    public static final double CAMERA_PITCH_UP_DEG = 28.538216;
    public static final double CAMERA_YAW_RIGHT_DEG = 0;
    public static final double SHOOTER_YAW_RIGHT_DEG = 0; // Team confirmed forward-firing shooter.

    // Keep disabled until the wheel branch has verified corners and rotation direction.
    // Used only by the separate Limelight Align Test, never by MainTeleOp.
    public static final boolean TEST_DRIVE_VERIFIED = false;
    public static final String[] TEST_MOTOR_NAMES = {"fL", "fR", "bL", "bR"};
    public static final boolean[] TEST_MOTOR_REVERSED = {false, true, false, true};
    public static final double TEST_DRIVE_LIMIT = 0.25;

    public static Settings settings() {
        return new Settings(CAMERA_MOUNT_VERIFIED, CAMERA_POSITION, SHOOTER_EXIT_POSITION,
                CAMERA_ROLL_CLOCKWISE_DEG, CAMERA_PITCH_UP_DEG, CAMERA_YAW_RIGHT_DEG,
                SHOOTER_YAW_RIGHT_DEG, new TagCalibration[] {
                    // Add measured entries, one for EACH ID you want to target. Example syntax:
                    // new TagCalibration(30, new Vector(right, down, out), new Vector(0, -1, 0))
                    // The opening vector is measured in that TAG's local native Limelight axes.
                    // The up vector is the cluster's printed-up direction expressed in that tag.
                    // No entries -> telemetry works, but automatic steering stays disabled.
                });
    }

    public enum Alliance { UNSELECTED, RED, BLUE }
    public enum Cell {
        RED_SCORING(Alliance.RED, 30), RED_AUDIENCE(Alliance.RED, 34),
        BLUE_AUDIENCE(Alliance.BLUE, 38), BLUE_SCORING(Alliance.BLUE, 42);
        public final Alliance alliance;
        private final int first;
        Cell(Alliance alliance, int first) { this.alliance = alliance; this.first = first; }
        public static Cell forTag(int id) {
            for (Cell cell : values()) if (id >= cell.first && id < cell.first + 4) return cell;
            return null;
        }
    }

    public static final class TagCalibration {
        public final int id;
        public final Vector openingInTag, clusterUpInTag;
        public TagCalibration(int id, Vector openingInTag, Vector clusterUpInTag) {
            if (Cell.forTag(id) == null || openingInTag == null || !openingInTag.finite()
                    || clusterUpInTag == null || !clusterUpInTag.finite()
                    || clusterUpInTag.length() < 0.9 || clusterUpInTag.length() > 1.1) {
                throw new IllegalArgumentException("Tag calibration needs BIOBUZZ ID, finite offset, unit up axis");
            }
            this.id = id;
            this.openingInTag = openingInTag;
            this.clusterUpInTag = clusterUpInTag.times(1 / clusterUpInTag.length());
        }
    }

    public static final class Settings {
        public final boolean mountVerified;
        public final Vector cameraPosition, shooterPosition;
        public final double roll, pitch, yaw, shooterYaw;
        private final TagCalibration[] tags;
        public final int pipeline = PIPELINE;
        public final double maxAgeMs = 200, minAreaPercent = 0.05;
        public final double minRangeM = 0.3, maxRangeM = 5.0;
        // Up-axis dot robot-up must be comfortably positive, away from upside-down boundary.
        public final double minUpDot = 0.15, maxTagDisagreementDeg = 5;
        public final double kp = 0.02, maxTurn = 0.25, turnSign = 1;
        public final double toleranceDeg = 1.5, maxSettledRateDegPerSec = 8;
        public final double settleMs = 200, overrideDeadband = 0.15;
        public final int acquisitionFrames = 3;

        public Settings(boolean verified, Vector camera, Vector shooter, double roll, double pitch,
                        double yaw, double shooterYaw, TagCalibration[] tags) {
            mountVerified = verified;
            cameraPosition = camera;
            shooterPosition = shooter;
            this.roll = roll; this.pitch = pitch; this.yaw = yaw; this.shooterYaw = shooterYaw;
            this.tags = tags.clone();
            for (int i = 0; i < this.tags.length; i++) {
                if (this.tags[i] == null) throw new IllegalArgumentException("Null tag calibration");
                for (int j = 0; j < i; j++) if (this.tags[j].id == this.tags[i].id) {
                    throw new IllegalArgumentException("Duplicate tag calibration: " + this.tags[i].id);
                }
            }
        }
        public boolean calibrated() {
            return mountVerified && cameraPosition != null && cameraPosition.finite()
                    && shooterPosition != null && shooterPosition.finite()
                    && Double.isFinite(roll) && Double.isFinite(pitch) && Double.isFinite(yaw)
                    && Double.isFinite(shooterYaw) && tags.length > 0;
        }
        public TagCalibration tag(int id) {
            for (TagCalibration tag : tags) if (tag.id == id) return tag;
            return null;
        }
    }
}
