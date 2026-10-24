package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignConfig.Alliance;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Frame;
import org.firstinspires.ftc.teamcode.vision.autoalign.AutoAlignController.Output;
import org.firstinspires.ftc.teamcode.vision.autoalign.LimelightAutoAlign;

/** Separate test mode. Leaves MainTeleOp, wheel diagnostics, and mechanisms untouched. */
@TeleOp(name = "Limelight Align Test", group = "Vision")
public final class LimelightAlignTest extends OpMode {
    private final AutoAlignController controller = new AutoAlignController(AutoAlignConfig.settings());
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private LimelightAutoAlign vision;
    private Alliance alliance = Alliance.UNSELECTED;
    private String connection = "Not initialized";
    private boolean controlsReleased;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        // Unverified test-drive settings leave the test completely vision-only.
        if (AutoAlignConfig.TEST_DRIVE_VERIFIED) {
            for (int i = 0; i < motors.length; i++) {
                motors[i] = hardwareMap.get(DcMotorEx.class, AutoAlignConfig.TEST_MOTOR_NAMES[i]);
                motors[i].setPower(0);
                motors[i].setDirection(AutoAlignConfig.TEST_MOTOR_REVERSED[i]
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        try {
            vision = new LimelightAutoAlign(hardwareMap);
            vision.start();
            connection = "Connected";
        } catch (RuntimeException e) {
            connection = "Limelight unavailable: " + e.getClass().getSimpleName();
        }
        telemetry.addLine("D-pad LEFT=RED / RIGHT=BLUE before Start");
        telemetry.addData("Limelight", connection);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left && !gamepad1.dpad_right) alliance = Alliance.RED;
        else if (gamepad1.dpad_right && !gamepad1.dpad_left) alliance = Alliance.BLUE;
        if (vision != null) vision.read();
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("D-pad LEFT=RED / RIGHT=BLUE. Start locks the selection.");
        showVision();
        telemetry.addData("Calibration", AutoAlignConfig.settings().calibrated() ? "Configured" : "REQUIRED");
        telemetry.addData("Test drive", AutoAlignConfig.TEST_DRIVE_VERIFIED ? "Enabled (25% limit)" : "DISABLED - vision only");
    }

    @Override
    public void start() {
        controller.reset();
        controlsReleased = false;
        stopMotors();
    }

    @Override
    public void loop() {
        Frame frame = vision == null ? null : vision.read();
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x);
        double manualTurn = deadband(gamepad1.right_stick_x);
        if (!gamepad1.right_bumper && forward == 0 && strafe == 0 && manualTurn == 0) controlsReleased = true;
        boolean held = controlsReleased && gamepad1.right_bumper;
        Output output = controller.update(frame, alliance, held, manualTurn,
                Math.hypot(forward, strafe), System.nanoTime());
        double turn = manualTurn;
        if (held && Math.abs(manualTurn) <= 0.15) turn = output.automatic ? output.turn : 0;
        if (AutoAlignConfig.TEST_DRIVE_VERIFIED && controlsReleased) {
            // Robot-centric test drive. An existing field-centric implementation is not modified.
            forward *= AutoAlignConfig.TEST_DRIVE_LIMIT;
            strafe *= AutoAlignConfig.TEST_DRIVE_LIMIT;
            if (!output.automatic) turn *= AutoAlignConfig.TEST_DRIVE_LIMIT;
            double[] powers = {forward + strafe + turn, forward - strafe - turn,
                    forward - strafe + turn, forward + strafe - turn};
            double max = AutoAlignConfig.TEST_DRIVE_LIMIT;
            for (double power : powers) max = Math.max(max, Math.abs(power));
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(AutoAlignConfig.TEST_DRIVE_LIMIT * powers[i] / max);
            }
        } else stopMotors();

        telemetry.addData("Alliance (locked)", alliance);
        telemetry.addData("Alignment", output.state);
        telemetry.addData("Cell", output.cell == null ? "none" : output.cell);
        telemetry.addData("Tags used", output.visibleTags);
        telemetry.addData("Heading error (deg)", "%.2f", output.errorDeg);
        telemetry.addData("Horizontal range (m)", "%.2f", output.rangeM);
        telemetry.addData("Turn request", "%.3f", output.turn);
        telemetry.addData("Heading aligned", output.aligned ? "YES - shooting calibration is separate" : "NO");
        telemetry.addData("Test drive", AutoAlignConfig.TEST_DRIVE_VERIFIED ? "Enabled" : "DISABLED - vision only");
        if (!controlsReleased) telemetry.addLine("Release R1 and center sticks once to enable controls.");
        telemetry.addLine("Hold R1 to align. Right stick overrides. Release R1 for manual steering.");
        showVision();
    }

    private void showVision() {
        telemetry.addData("Limelight", vision == null ? connection : vision.status());
        telemetry.addData("Visible IDs", vision == null ? "none" : vision.visibleIds());
        telemetry.addData("Vision age including latency (ms)", "%.0f", vision == null ? Double.NaN : vision.ageMs());
    }
    private static double deadband(double x) { return Math.abs(x) < 0.08 ? 0 : x; }
    private void stopMotors() { for (DcMotorEx motor : motors) if (motor != null) motor.setPower(0); }
    @Override
    public void stop() {
        try { stopMotors(); }
        finally { if (vision != null) vision.stop(); }
    }
}
