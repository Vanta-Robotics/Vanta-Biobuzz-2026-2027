package org.firstinspires.ftc.teamcode.vision.autoalign;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.autoalign.AlignGeometry.Vector;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Frame;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Observation;

/** FTC SDK adapter. Uses each selected tag's pose, never the global primary-target tx. */
public final class LimelightAutoAlign {
    private final Limelight3A camera;
    private String status = "Not started", visible = "none";
    private double ageMs = Double.NaN;

    public LimelightAutoAlign(HardwareMap map) {
        camera = map.get(Limelight3A.class, AutoAlignConfig.LIMELIGHT_NAME);
    }
    public void start() {
        camera.setPollRateHz(100);
        camera.pipelineSwitch(AutoAlignConfig.PIPELINE);
        camera.start();
        status = "Polling";
    }
    public Frame read() {
        visible = "none";
        ageMs = Double.NaN;
        try {
            LLResult result = camera.getLatestResult();
            if (result == null) { status = "Waiting for Limelight data"; return null; }
            // getStaleness is receive age, not total exposure-to-control latency.
            ageMs = result.getStaleness() + result.getCaptureLatency()
                    + result.getTargetingLatency() + result.getParseLatency();
            List<Observation> observations = new ArrayList<>();
            StringBuilder ids = new StringBuilder();
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (ids.length() > 0) ids.append(", ");
                ids.append(tag.getFiducialId());
                Pose3D pose = tag.getTargetPoseCameraSpace();
                if (pose == null) continue;
                Position p = pose.getPosition().toUnit(DistanceUnit.METER);
                YawPitchRollAngles r = pose.getOrientation();
                // The FTC Limelight adapter preserves the native t6t_cs axes and RX/RY/RZ.
                observations.add(new Observation(tag.getFiducialId(), tag.getTargetArea(),
                        new Vector(p.x, p.y, p.z), r.getRoll(AngleUnit.DEGREES),
                        r.getPitch(AngleUnit.DEGREES), r.getYaw(AngleUnit.DEGREES)));
            }
            visible = ids.length() == 0 ? "none" : ids.toString();
            status = result.isValid() ? "Receiving vision" : "No valid target";
            return new Frame(result.isValid(), result.getPipelineIndex(), result.getTimestamp(), ageMs, observations);
        } catch (RuntimeException e) {
            status = "Vision read failed: " + e.getClass().getSimpleName();
            return null;
        }
    }
    public String status() { return status; }
    public String visibleIds() { return visible; }
    public double ageMs() { return ageMs; }
    public void stop() { camera.stop(); status = "Stopped"; }
}
