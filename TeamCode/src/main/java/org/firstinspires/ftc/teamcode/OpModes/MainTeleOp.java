package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime; // Added for the D-pad button timer

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private Shooter pollenShooter;
    private Intake intake;
    private ShooterStopper shooterStopper;
    private IMU imu;


    private double tuningKp = 0.2; // Start with your original 0.2 default value
    private ElapsedTime buttonTimer = new ElapsedTime(); // Prevents number from rocketing up instantly

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        backLeft = hardwareMap.get(DcMotorEx.class, "bL");
        backRight = hardwareMap.get(DcMotorEx.class, "bR");

        pollenShooter = new Shooter(hardwareMap, "sh1", 0.5);
        intake = new Intake(hardwareMap, "in1");
        shooterStopper = new ShooterStopper(hardwareMap, "stopperA", 0, 0.5);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

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

        buttonTimer.reset(); // Clear button timer on initialization

        telemetry.addData("Drive", "Left stick moves on the field, right stick turns");
        telemetry.addData("Heading", "Face field forward before Start; Back resets heading");
        telemetry.addData("Mechanisms", "A/B intake, X/Y stopper, right trigger shooter");
    }

    @Override
    public void start() {
        imu.resetYaw();
    }

    @Override
    public void loop() {
        // 🔄 1. D-PAD CONTROLS: Change KP safely in an active loop system
        if (buttonTimer.milliseconds() > 200) {
            if (gamepad1.dpad_up) {
                tuningKp += 0.05; // Adds 0.05 per tap
                buttonTimer.reset();
            } else if (gamepad1.dpad_down) {
                tuningKp = Math.max(0.0, tuningKp - 0.05); // Prevents KP from dropping below 0
                buttonTimer.reset();
            }
        }

        try {
            java.lang.reflect.Field kpField = org.firstinspires.ftc.teamcode.OpModes.Shooter.class.getDeclaredField("KP");
            kpField.setAccessible(true);
            kpField.set(null, tuningKp);
        } catch (Exception e) {
            // Failsafe
        }

        intake.update(gamepad1.a, gamepad1.b);
        shooterStopper.update(gamepad1.x, gamepad1.y);
        pollenShooter.update(gamepad1.right_trigger);

        // Live feedback right alongside your original data lines
        telemetry.addData("Live Tuning KP (D-pad Up/Down)", "%.2f", tuningKp);
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

        double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double robotStrafe = strafe * Math.cos(heading) + forward * Math.sin(heading);
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(heading));

        double fl = robotForward + robotStrafe + turn;
        double fr = robotForward - robotStrafe - turn;
        double bl = robotForward - robotStrafe + turn;
        double br = robotForward + robotStrafe - turn;

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
