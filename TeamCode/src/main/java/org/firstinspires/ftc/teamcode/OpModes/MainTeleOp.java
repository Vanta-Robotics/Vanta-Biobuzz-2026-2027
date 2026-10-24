package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private Shooter pollenShooter;
    private Intake intake;
    private IMU imu;


    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotorEx.class, "bL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
        backRight = hardwareMap.get(DcMotorEx.class, "bR");
        // Full trigger requests 50% of the configured motor's achievable speed.
        // Gate positions still need checking against the stopper linkage.
        pollenShooter = new Shooter(hardwareMap, "sh1", 0.5, "stopperA", 0, 0.5);
        intake = new Intake(hardwareMap, "in1", "intakeGate");

        // The hub sits on the right side, with its logo facing right and USB ports up.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Keep the directions from the current robot code.
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] driveMotors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
        telemetry.addData("Drive", "Left stick moves on the field, right stick turns");
        telemetry.addData("Heading", "Face field forward before Start; Back resets heading");
        telemetry.addData("Mechanisms", "A/B intake, X/Y stopper, right trigger shooter");
        telemetry.addData("Intake gate", "Hold left bumper to open");
    }

    @Override
    public void start() {
        // The direction we're facing at Start becomes field forward.
        imu.resetYaw();
    }

    @Override
    public void loop() {
        intake.update(gamepad1.a, gamepad1.b, gamepad1.left_bumper);
        pollenShooter.updateGate(gamepad1.x, gamepad1.y);
        pollenShooter.update(gamepad1.right_trigger);
        telemetry.addData("Shooter target (ticks/s)", "%.0f",
                pollenShooter.getTargetVelocityTicksPerSecond());
        telemetry.addData("Shooter actual (ticks/s)", "%.0f",
                pollenShooter.getVelocityTicksPerSecond());
        telemetry.addData("Shooter PID power", "%.2f", pollenShooter.getOutputPower());

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        if (Math.abs(forward) < 0.08) forward = 0;
        if (Math.abs(strafe) < 0.08) strafe = 0;
        if (Math.abs(turn) < 0.08) turn = 0;

        if (gamepad1.back) imu.resetYaw();
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the field's forward/right commands into the robot's forward/right.
        //double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        //double robotStrafe = strafe * Math.cos(heading) + forward * Math.sin(heading);
        double robotForward = forward;
        double robotStrafe = strafe;
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(heading));

        double den = Math.max(Math.abs(robotForward) + Math.abs(robotStrafe) + Math.abs(turn), 1.0);

        double fl = (robotForward + robotStrafe + turn)/den;
        double fr = (robotForward - robotStrafe - turn)/den;
        double bl = (robotForward - robotStrafe + turn)/den;
        double br = (robotForward + robotStrafe - turn)/den;

        // Scale the powers together so diagonal driving keeps its direction.
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));

        frontLeft.setPower((fl / max));
        frontRight.setPower((fr / max));
        backLeft.setPower(bl / max);
        backRight.setPower((br / max));

         /*
        frontLeft.setPower(gamepad1.dpad_up ? 0.2 : 0);
        frontRight.setPower(gamepad1.dpad_right ? 0.2 : 0);
        backLeft.setPower(gamepad1.dpad_left ? 0.2 : 0);
        backRight.setPower(gamepad1.dpad_down ? 0.2 : 0);
        */
        telemetry.addData("front Left", frontLeft.getPower());
        telemetry.addData("back Left", backLeft.getPower());
        telemetry.addData("front right", frontRight.getPower());
        telemetry.addData("back right", backRight.getPower());

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
