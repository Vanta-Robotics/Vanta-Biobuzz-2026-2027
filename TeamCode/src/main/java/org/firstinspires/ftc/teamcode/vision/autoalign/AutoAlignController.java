package org.firstinspires.ftc.teamcode.vision.autoalign;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.vision.autoalign.AlignGeometry.Vector;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Alliance;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Cell;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Settings;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.TagCalibration;

/** Produces a clockwise/right-positive turn demand; never writes to any motor. */
public final class AutoAlignController {
    public static final class Observation {
        public final int id;
        public final double area, rx, ry, rz;
        public final Vector position;
        public Observation(int id, double area, Vector position, double rx, double ry, double rz) {
            this.id = id; this.area = area; this.position = position;
            this.rx = rx; this.ry = ry; this.rz = rz;
        }
    }
    public static final class Frame {
        public final boolean valid;
        public final int pipeline;
        public final double timestamp, ageMs;
        public final List<Observation> observations;
        public Frame(boolean valid, int pipeline, double timestamp, double ageMs, List<Observation> items) {
            this.valid = valid; this.pipeline = pipeline; this.timestamp = timestamp; this.ageMs = ageMs;
            observations = Collections.unmodifiableList(new ArrayList<>(items));
        }
    }
    public static final class Output {
        public final String state;
        public final Cell cell;
        public final int visibleTags;
        public final double turn, errorDeg, rangeM;
        public final boolean automatic, aligned;
        private Output(String state, Cell cell, int tags, double turn, double error, double range,
                       boolean automatic, boolean aligned) {
            this.state = state; this.cell = cell; visibleTags = tags;
            this.turn = turn; errorDeg = error; rangeM = range;
            this.automatic = automatic; this.aligned = aligned;
        }
    }
    private static final class Aim {
        final Cell cell;
        final double error, range;
        final int tags;
        Aim(Cell cell, double error, double range, int tags) {
            this.cell = cell; this.error = error; this.range = range; this.tags = tags;
        }
    }

    private final Settings config;
    private Cell lockedCell;
    private Alliance lastAlliance = Alliance.UNSELECTED;
    private double sourceTimestamp = Double.NaN;
    private long sourceChangedAt = -1, lastSampleAt = -1, insideSince = -1;
    private int acquiredFrames, settledFrames;
    private double lastError, filteredError, errorRate = Double.POSITIVE_INFINITY;

    public AutoAlignController(Settings config) { this.config = config; }

    public void reset() {
        clearTarget();
        sourceTimestamp = Double.NaN;
        sourceChangedAt = -1;
        lastAlliance = Alliance.UNSELECTED;
    }
    private void clearTarget() {
        lockedCell = null; acquiredFrames = 0; settledFrames = 0;
        lastSampleAt = -1; insideSince = -1;
        errorRate = Double.POSITIVE_INFINITY;
    }
    private Output blocked(String state) {
        clearTarget();
        return new Output(state, null, 0, 0, Double.NaN, Double.NaN, false, false);
    }

    public Output update(Frame frame, Alliance alliance, boolean held, double manualTurn,
                         double translationDemand, long nowNanos) {
        if (alliance == null || alliance == Alliance.UNSELECTED) return blocked("SELECT ALLIANCE BEFORE START");
        if (lastAlliance != alliance) { clearTarget(); lastAlliance = alliance; }
        if (!Double.isFinite(manualTurn) || !Double.isFinite(translationDemand)) return blocked("INVALID DRIVER INPUT");
        if (!config.calibrated()) return blocked("CALIBRATION REQUIRED");
        if (!held) return blocked("MANUAL - HOLD R1 TO ALIGN");
        if (Math.abs(manualTurn) > config.overrideDeadband) return blocked("DRIVER TURN OVERRIDE");
        if (frame == null || !frame.valid) return blocked("NO VALID VISION");
        if (frame.pipeline != config.pipeline) return blocked("WRONG PIPELINE");
        if (!Double.isFinite(frame.ageMs) || frame.ageMs < 0 || frame.ageMs > config.maxAgeMs
                || !Double.isFinite(frame.timestamp) || frame.timestamp <= 0) return blocked("STALE/INVALID VISION");

        boolean newFrame = Double.compare(frame.timestamp, sourceTimestamp) != 0;
        if (newFrame) {
            // A camera restart or clock rollback also requires a new target acquisition.
            if (Double.isFinite(sourceTimestamp) && frame.timestamp < sourceTimestamp) clearTarget();
            sourceTimestamp = frame.timestamp;
            sourceChangedAt = nowNanos;
        }
        if (sourceChangedAt < 0 || nowNanos < sourceChangedAt
                || (nowNanos - sourceChangedAt) / 1e6 > config.maxAgeMs) return blocked("CAMERA FRAME FROZEN");

        List<Aim> candidates = candidates(frame, alliance);
        Aim selected = null;
        for (Aim aim : candidates) if (aim.cell == lockedCell) selected = aim;
        if (selected == null) {
            for (Aim aim : candidates) if (selected == null
                    || Math.abs(aim.error) < Math.abs(selected.error)) selected = aim;
        }
        if (selected == null) return blocked("NO CALIBRATED UPRIGHT ALLIANCE CELL");
        if (selected.cell != lockedCell) {
            clearTarget();
            lockedCell = selected.cell;
            // Do not acquire a different target from an already-consumed image.
            if (!newFrame) return new Output("WAITING FOR NEW FRAME", lockedCell, selected.tags,
                    0, selected.error, selected.range, false, false);
        }
        if (newFrame) {
            double dt = lastSampleAt < 0 ? 0 : (nowNanos - lastSampleAt) / 1e9;
            errorRate = dt > 0 ? Math.abs(wrap(selected.error - lastError) / dt) : Double.POSITIVE_INFINITY;
            // Reject a large discontinuity (e.g. a tipping cell or inconsistent pose).
            if (lastSampleAt >= 0 && Math.abs(wrap(selected.error - lastError)) > 12) {
                acquiredFrames = 0; settledFrames = 0; insideSince = -1;
            }
            filteredError = acquiredFrames == 0 ? selected.error
                    : wrap(filteredError + 0.5 * wrap(selected.error - filteredError));
            // A lagging filter must never steer away after crossing the target bearing.
            if (Math.signum(filteredError) != Math.signum(selected.error)) filteredError = selected.error;
            acquiredFrames++;
            lastError = selected.error;
            lastSampleAt = nowNanos;
            if (Math.abs(selected.error) <= config.toleranceDeg
                    && errorRate <= config.maxSettledRateDegPerSec && Math.abs(translationDemand) <= 0.08) {
                if (insideSince < 0) insideSince = nowNanos;
                settledFrames++;
            } else { insideSince = -1; settledFrames = 0; }
        }
        if (Math.abs(translationDemand) > 0.08) { insideSince = -1; settledFrames = 0; }
        if (acquiredFrames < config.acquisitionFrames) return new Output("ACQUIRING", lockedCell,
                selected.tags, 0, selected.error, selected.range, false, false);

        boolean aligned = insideSince >= 0 && settledFrames >= 3
                && (lastSampleAt - insideSince) / 1e6 >= config.settleMs;
        // Stop immediately on entering the raw tolerance band, even if the filter lags.
        double turn = Math.abs(selected.error) <= config.toleranceDeg ? 0
                : clamp(config.kp * filteredError * config.turnSign, config.maxTurn);
        return new Output(aligned ? "ALIGNED (HEADING ONLY)" : "ALIGNING", lockedCell,
                selected.tags, turn, selected.error, selected.range, true, aligned);
    }

    private List<Aim> candidates(Frame frame, Alliance alliance) {
        List<Aim> result = new ArrayList<>();
        for (Cell cell : Cell.values()) {
            if (cell.alliance != alliance) continue;
            List<Vector> points = new ArrayList<>();
            for (Observation o : frame.observations) {
                if (o == null || Cell.forTag(o.id) != cell || !Double.isFinite(o.area)
                        || o.area < config.minAreaPercent || o.position == null || !o.position.finite()
                        || o.position.z <= 0 || !Double.isFinite(o.rx)
                        || !Double.isFinite(o.ry) || !Double.isFinite(o.rz)) continue;
                TagCalibration tag = config.tag(o.id);
                if (tag == null) continue;
                Vector up = robotVector(AlignGeometry.rotate(tag.clusterUpInTag, o.rx, o.ry, o.rz));
                if (up.z < config.minUpDot) continue;
                Vector nativePoint = o.position.plus(AlignGeometry.rotate(tag.openingInTag, o.rx, o.ry, o.rz));
                if (nativePoint.z <= 0) continue;
                Vector fromShooter = robotVector(nativePoint).plus(config.cameraPosition).minus(config.shooterPosition);
                double range = Math.hypot(fromShooter.x, fromShooter.y);
                if (!fromShooter.finite() || range < config.minRangeM || range > config.maxRangeM
                        || fromShooter.y <= 0) continue;
                points.add(fromShooter);
            }
            if (points.isEmpty()) continue;
            Vector mean = new Vector(0, 0, 0);
            for (Vector point : points) mean = mean.plus(point);
            mean = mean.times(1.0 / points.size());
            double bearing = Math.toDegrees(Math.atan2(mean.x, mean.y));
            boolean agree = true;
            for (Vector point : points) if (Math.abs(wrap(Math.toDegrees(Math.atan2(point.x, point.y))
                    - bearing)) > config.maxTagDisagreementDeg) agree = false;
            if (agree) result.add(new Aim(cell, wrap(bearing - config.shooterYaw),
                    Math.hypot(mean.x, mean.y), points.size()));
        }
        return result;
    }
    private Vector robotVector(Vector v) {
        return AlignGeometry.cameraToRobot(v, config.roll, config.pitch, config.yaw);
    }
    private static double wrap(double degrees) {
        double value = degrees % 360;
        return value > 180 ? value - 360 : value < -180 ? value + 360 : value;
    }
    private static double clamp(double value, double max) { return Math.max(-max, Math.min(max, value)); }
}
