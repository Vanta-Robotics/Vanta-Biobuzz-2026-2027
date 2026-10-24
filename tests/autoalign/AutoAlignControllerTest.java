import java.util.Arrays;
import org.firstinspires.ftc.teamcode.vision.autoalign.AlignGeometry;
import org.firstinspires.ftc.teamcode.vision.autoalign.AlignGeometry.Vector;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Alliance;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Cell;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Settings;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.TagCalibration;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Frame;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Observation;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Output;

/** Behavioral tests of the real controller and geometry, without an Android runtime. */
public final class AutoAlignControllerTest {
    private static final Vector ZERO = new Vector(0, 0, 0);
    private static int checks;
    private static long time(int ms) { return 1_000_000_000L + ms * 1_000_000L; }
    private static void check(boolean condition, String reason) {
        checks++;
        if (!condition) throw new AssertionError(reason);
    }
    private static void close(double actual, double expected, String reason) {
        check(Math.abs(actual - expected) < 1e-6, reason + ": " + actual + " != " + expected);
    }
    private static TagCalibration tag(int id) { return new TagCalibration(id, ZERO, new Vector(0, -1, 0)); }
    private static Settings settings(TagCalibration... tags) {
        return new Settings(true, ZERO, ZERO, 0, 0, 0, 0, tags);
    }
    private static Settings standard() { return settings(tag(30), tag(31), tag(34), tag(38), tag(42)); }
    private static Observation observed(int id, double bearing) {
        return new Observation(id, 2, new Vector(2 * Math.tan(Math.toRadians(bearing)), 0, 2), 0, 0, 0);
    }
    private static Frame frame(double timestamp, Observation... tags) {
        return new Frame(true, 0, timestamp, 20, Arrays.asList(tags));
    }
    private static Output step(AutoAlignController c, Frame f, int ms) {
        return c.update(f, Alliance.RED, true, 0, 0, time(ms));
    }
    private static Output acquire(AutoAlignController c, Observation... tags) {
        step(c, frame(1, tags), 0);
        step(c, frame(2, tags), 50);
        return step(c, frame(3, tags), 100);
    }
    private static void stopped(Output output, String reason) {
        close(output.turn, 0, reason + " turn");
        check(!output.automatic && !output.aligned, reason + " state");
    }

    public static void main(String[] args) {
        check(Cell.forTag(30) == Cell.RED_SCORING && Cell.forTag(33) == Cell.RED_SCORING, "Red scoring IDs");
        check(Cell.forTag(34) == Cell.RED_AUDIENCE && Cell.forTag(37) == Cell.RED_AUDIENCE, "Red audience IDs");
        check(Cell.forTag(38) == Cell.BLUE_AUDIENCE && Cell.forTag(41) == Cell.BLUE_AUDIENCE, "Blue audience IDs");
        check(Cell.forTag(42) == Cell.BLUE_SCORING && Cell.forTag(45) == Cell.BLUE_SCORING, "Blue scoring IDs");
        check(Cell.forTag(29) == null && Cell.forTag(46) == null, "Unknown IDs excluded");
        check(!AutoAlignConfig.settings().calibrated(), "Unmeasured defaults must not enable steering");
        stopped(acquire(new AutoAlignController(AutoAlignConfig.settings()), observed(30, 10)), "Uncalibrated mount");

        AutoAlignController controller = new AutoAlignController(standard());
        stopped(controller.update(frame(1, observed(30, 10)), Alliance.UNSELECTED, true, 0, 0, time(0)), "No alliance");
        stopped(controller.update(frame(1, observed(30, 10)), Alliance.RED, false, 0, 0, time(0)), "Released R1");
        stopped(controller.update(frame(1, observed(30, 10)), Alliance.RED, true, 0.3, 0, time(0)), "Manual turn override");
        stopped(acquire(new AutoAlignController(standard()), observed(38, 10)), "Opposing alliance");
        stopped(acquire(new AutoAlignController(settings(tag(30))), observed(31, 10)), "Uncalibrated ID");
        stopped(acquire(new AutoAlignController(standard()), observed(99, 10)), "Unknown target");
        stopped(acquire(new AutoAlignController(standard()),
                new Observation(30, 2, new Vector(0, 0, 2), 0, 0, 180)), "Upside-down cell");
        stopped(acquire(new AutoAlignController(standard()),
                new Observation(30, 0.001, new Vector(0, 0, 2), 0, 0, 0)), "Tiny target");
        stopped(acquire(new AutoAlignController(standard()),
                new Observation(30, 2, new Vector(0, 0, -2), 0, 0, 0)), "Behind camera");
        stopped(acquire(new AutoAlignController(standard()),
                new Observation(30, 2, new Vector(Double.NaN, 0, 2), 0, 0, 0)), "Non-finite pose");
        stopped(acquire(new AutoAlignController(standard()),
                new Observation(30, 2, new Vector(0, 0, 8), 0, 0, 0)), "Out of calibrated range");

        Output right = acquire(new AutoAlignController(standard()), observed(30, 20));
        check(right.automatic && right.turn > 0 && !right.aligned, "Right target commands right turn");
        close(right.turn, 0.25, "Turn cap");
        Output left = acquire(new AutoAlignController(standard()), observed(30, -10));
        check(left.automatic && left.turn < 0, "Left target commands left turn");
        controller = new AutoAlignController(standard());
        acquire(controller, observed(30, 10));
        Output crossed = step(controller, frame(4, observed(30, -2)), 150);
        check(crossed.turn < 0, "Filtered turn reverses immediately when the target crosses the aim line");
        Output blue = null;
        controller = new AutoAlignController(standard());
        for (int i = 1; i <= 3; i++) blue = controller.update(frame(i, observed(38, 5)),
                Alliance.BLUE, true, 0, 0, time((i - 1) * 50));
        check(blue.automatic && blue.cell == Cell.BLUE_AUDIENCE, "Blue target accepted when selected");

        for (double age : new double[] {-1, 201, Double.NaN}) {
            stopped(step(new AutoAlignController(standard()), new Frame(true, 0, 1, age,
                    Arrays.asList(observed(30, 10))), 0), "Invalid age " + age);
        }
        stopped(step(new AutoAlignController(standard()), new Frame(true, 1, 1, 20,
                Arrays.asList(observed(30, 10))), 0), "Wrong pipeline");
        stopped(step(new AutoAlignController(standard()), new Frame(false, 0, 1, 20,
                Arrays.asList(observed(30, 10))), 0), "Invalid result");
        stopped(step(new AutoAlignController(standard()), frame(0, observed(30, 10)), 0), "Missing source timestamp");

        controller = new AutoAlignController(standard());
        Frame same = frame(1, observed(30, 10));
        step(controller, same, 0);
        stopped(step(controller, same, 50), "Repeated image does not acquire");
        stopped(step(controller, same, 100), "Repeated image still does not acquire");
        check(step(controller, same, 201).state.equals("CAMERA FRAME FROZEN"), "Frozen camera detected even with fake fresh receive age");

        controller = new AutoAlignController(standard());
        Output center = null;
        for (int i = 0; i <= 6; i++) center = step(controller, frame(i + 1, observed(30, 0)), i * 50);
        check(center.aligned && center.automatic && center.turn == 0, "Fresh settled images produce heading-aligned");
        stopped(step(controller, null, 350), "Target loss immediately clears steering and alignment");
        controller = new AutoAlignController(standard());
        for (int i = 0; i <= 3; i++) step(controller, frame(i + 1, observed(30, 0)), i * 50);
        check(!step(controller, frame(4, observed(30, 0)), 300).aligned, "Re-reading an image cannot finish the settling timer");
        controller = new AutoAlignController(standard());
        for (int i = 0; i <= 8; i++) center = controller.update(frame(i + 1, observed(30, 0)),
                Alliance.RED, true, 0, 0.5, time(i * 50));
        check(!center.aligned, "Translation command prevents settled indication");

        controller = new AutoAlignController(standard());
        acquire(controller, observed(30, 10), observed(34, 15));
        Output retained = step(controller, frame(4, observed(30, 10), observed(34, 1)), 150);
        check(retained.cell == Cell.RED_SCORING, "Keep eligible locked cell instead of jumping to closer tag");
        Output switched = step(controller, frame(5, observed(34, 1)), 200);
        check(switched.cell == Cell.RED_AUDIENCE && !switched.automatic && !switched.aligned,
                "New cell must reacquire");
        stopped(acquire(new AutoAlignController(standard()), observed(30, 12), observed(31, -12)),
                "Inconsistent same-cell poses rejected");

        controller = new AutoAlignController(standard());
        acquire(controller, observed(30, 4));
        Output jumped = step(controller, frame(4, observed(30, 25)), 150);
        check(!jumped.automatic && !jumped.aligned, "Large target jump suspends turn while reacquiring");
        Output rollback = step(controller, frame(1, observed(30, 4)), 200);
        check(!rollback.automatic, "Camera timestamp rollback reacquires");

        TagCalibration offset = new TagCalibration(30, new Vector(0.2, 0, 0), new Vector(0, -1, 0));
        Output geometry = acquire(new AutoAlignController(settings(offset)), observed(30, 0));
        close(geometry.errorDeg, Math.toDegrees(Math.atan2(0.2, 2)), "Aim at offset opening instead of tag center");
        Output rotated = acquire(new AutoAlignController(settings(offset)),
                new Observation(30, 2, new Vector(0, 0, 2), 0, 180, 0));
        close(rotated.errorDeg, -geometry.errorDeg, "Rotate tag-to-opening offset with target pose");
        Settings sideMount = new Settings(true, new Vector(0.2, 0, 0), ZERO, 0, 0, 0, 0,
                new TagCalibration[] {tag(30)});
        close(acquire(new AutoAlignController(sideMount), observed(30, 0)).errorDeg,
                geometry.errorDeg, "Camera lateral offset compensation");
        Settings shooterOffset = new Settings(true, ZERO, new Vector(0.2, 0, 0), 0, 0, 0, 0,
                new TagCalibration[] {tag(30)});
        close(acquire(new AutoAlignController(shooterOffset), observed(30, 0)).errorDeg,
                -geometry.errorDeg, "Shooter lateral offset compensation");
        Settings yawed = new Settings(true, ZERO, ZERO, 0, 0, 10, 0, new TagCalibration[] {tag(30)});
        close(acquire(new AutoAlignController(yawed), observed(30, 0)).errorDeg, 10, "Camera yaw compensation");
        Vector pitched = AlignGeometry.cameraToRobot(new Vector(0, 0, 2), 0, 45, 0);
        close(pitched.y, Math.sqrt(2), "Upward camera pitch forward projection");
        close(pitched.z, Math.sqrt(2), "Upward camera pitch height projection");

        // Independent synthetic fixture using the measured camera mount. The opening
        // is at aiming-frame (0, 2, 1) m, observed through the 28.538216-degree camera.
        // Wheel centers below are deliberately TEST reference origins, not real exits.
        Observation cadTarget = new Observation(30, 2,
                new Vector(0.0031791, 0.2349753799319838, 1.9721163145904197), 0, 0, 0);
        Settings cadLeftReference = new Settings(true, AutoAlignConfig.CAMERA_POSITION,
                new Vector(-0.065825795, 0.07243985, 0.30818390),
                AutoAlignConfig.CAMERA_ROLL_CLOCKWISE_DEG, AutoAlignConfig.CAMERA_PITCH_UP_DEG,
                AutoAlignConfig.CAMERA_YAW_RIGHT_DEG, 0, new TagCalibration[] {tag(30)});
        Settings cadRightReference = new Settings(true, AutoAlignConfig.CAMERA_POSITION,
                new Vector(0.069370205, 0.06511985, 0.30818390),
                AutoAlignConfig.CAMERA_ROLL_CLOCKWISE_DEG, AutoAlignConfig.CAMERA_PITCH_UP_DEG,
                AutoAlignConfig.CAMERA_YAW_RIGHT_DEG, 0, new TagCalibration[] {tag(30)});
        Output cadLeft = acquire(new AutoAlignController(cadLeftReference), cadTarget);
        Output cadRight = acquire(new AutoAlignController(cadRightReference), cadTarget);
        close(cadLeft.errorDeg, 1.9558793675246273, "CAD pitch and left reference parallax");
        close(cadRight.errorDeg, -2.0533149261830794, "CAD pitch and right reference parallax");
        check(cadLeft.turn > 0 && cadRight.turn < 0,
                "Same camera observation needs opposite corrections for separate shooter origins");
        check(!AutoAlignConfig.SHOOTER_EXIT_POSITION.finite(),
                "CAD wheel centers must not silently become calibrated ball release points");

        boolean duplicateRejected = false;
        try { settings(tag(30), tag(30)); }
        catch (IllegalArgumentException expected) { duplicateRejected = true; }
        check(duplicateRejected, "Duplicate calibrations are rejected");
        System.out.println("PASS: " + checks + " auto-align behavior and geometry checks");
    }
}
