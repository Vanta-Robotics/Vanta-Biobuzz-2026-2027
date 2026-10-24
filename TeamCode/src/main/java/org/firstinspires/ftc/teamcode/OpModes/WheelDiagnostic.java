package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Tests configuration names directly, independently of MainTeleOp's assignments.
 * Support the robot with all wheels off the ground and clear of hands/objects.
 * Release R1 and all face buttons after Start to arm the test.
 * Hold R1 + exactly one face button to run one motor at 30% power:
 * Square=fL, Circle=fR, Cross=bL, Triangle=bR. Release R1 to stop.
 * Also hold L1 to test that same motor in reverse.
 * No IMU, intake, shooter, or servo is initialized by this OpMode.
 */
@TeleOp(name = "Wheel Test - One at a Time", group = "Diagnostics")
public final class WheelDiagnostic extends OpMode {
    private static final double TEST_POWER = 0.30;
    private final String[] names = {"fL", "fR", "bL", "bR"};
    private final String[] nominalCorners = {
            "front left", "front right", "back left", "back right"
    };
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final int[] startingTicks = new int[4];
    private boolean armed;

    @Override
    public void init() {
        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setPower(0);
            // Match the current MainTeleOp's directions, but NOT its left-name swap.
            motors[i].setDirection(i == 0 || i == 2
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            startingTicks[i] = motors[i].getCurrentPosition();
        }
        telemetry.setMsTransmissionInterval(100);
        showInstructions();
    }

    @Override
    public void init_loop() {
        showInstructions();
        telemetry.addLine("Motors stay stopped until Start and the test controls are used.");
    }

    @Override
    public void start() {
        armed = false;
        stopMotors();
    }

    @Override
    public void loop() {
        // FTC's x/b/a/y fields correspond to PS5 Square/Circle/Cross/Triangle.
        boolean[] buttons = {gamepad1.x, gamepad1.b, gamepad1.a, gamepad1.y};
        int pressedCount = 0;
        int selected = -1;
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i]) {
                pressedCount++;
                selected = i;
            }
        }

        if (!gamepad1.right_bumper && pressedCount == 0) armed = true;
        boolean running = armed && gamepad1.right_bumper && pressedCount == 1;
        double power = gamepad1.left_bumper ? -TEST_POWER : TEST_POWER;

        // Stop every unselected motor, including when changing button selections.
        for (int i = 0; i < motors.length; i++) {
            if (!running || i != selected) motors[i].setPower(0);
        }
        if (running) motors[selected].setPower(power);

        showInstructions();
        if (!armed) {
            telemetry.addData("Status", "Release R1 AND all face buttons once to arm");
        } else if (pressedCount > 1) {
            telemetry.addData("Status", "STOPPED: use only ONE face button");
        } else if (running) {
            telemetry.addData("Status", "RUNNING configuration name %s", names[selected]);
            telemetry.addData("Name suggests", nominalCorners[selected]);
            telemetry.addData("Test direction", gamepad1.left_bumper ? "REVERSE" : "POSITIVE");
            telemetry.addLine("Watch which PHYSICAL wheel moves. Does it match the name?");
        } else {
            telemetry.addData("Status", "STOPPED - hold R1 + one face button to test");
        }

        telemetry.addLine("--- Power, encoder speed, and ticks since Init ---");
        for (int i = 0; i < motors.length; i++) {
            telemetry.addData(names[i], "power %.2f | speed %.0f ticks/s | ticks %d",
                    running && i == selected ? power : 0,
                    motors[i].getVelocity(),
                    motors[i].getCurrentPosition() - startingTicks[i]);
        }
        telemetry.addLine("Wheel motion is the mapping check; encoder sign alone is not.");
    }

    private void showInstructions() {
        telemetry.addLine("WHEELS OFF GROUND. Keep hands and objects clear.");
        telemetry.addLine("R1 + Square = fL (nominal front left)");
        telemetry.addLine("R1 + Circle = fR (nominal front right)");
        telemetry.addLine("R1 + Cross = bL (nominal back left)");
        telemetry.addLine("R1 + Triangle = bR (nominal back right)");
        telemetry.addLine("Release R1 to STOP. Add L1 to reverse the selected wheel.");
    }

    private void stopMotors() {
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setPower(0);
        }
    }

    @Override
    public void stop() {
        armed = false;
        stopMotors();
    }
}
