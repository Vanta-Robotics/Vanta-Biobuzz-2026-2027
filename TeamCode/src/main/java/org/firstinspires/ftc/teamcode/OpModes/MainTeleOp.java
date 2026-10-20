package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private Shooter pollenShooter;
    private Intake intake;
    private ShooterStopper shooterStopper;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        backLeft = hardwareMap.get(DcMotorEx.class, "bL");
        backRight = hardwareMap.get(DcMotorEx.class, "bR");
        pollenShooter = new Shooter(hardwareMap, "sh1", 0.5);
        intake = new Intake(hardwareMap, "in1");
        // Check these positions against the stopper linkage before driving.
        shooterStopper = new ShooterStopper(hardwareMap, "stopperA", 0, 0.5);

        // Keep the directions from the current robot code.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] driveMotors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
        telemetry.addData("Drive", "Left stick moves, right stick turns");
        telemetry.addData("Mechanisms", "A/B intake, X/Y stopper, right trigger shooter");
    }

    @Override
    public void loop() {
        intake.update(gamepad1.a, gamepad1.b);
        shooterStopper.update(gamepad1.x, gamepad1.y);
        pollenShooter.update(gamepad1.right_trigger);

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        if (Math.abs(forward) < 0.08) forward = 0;
        if (Math.abs(strafe) < 0.08) strafe = 0;
        if (Math.abs(turn) < 0.08) turn = 0;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Scale the powers together so diagonal driving keeps its direction.
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    @Override
    public void stop() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
        if (pollenShooter != null) pollenShooter.stop();
        if (intake != null) intake.stop();
    }
}
