package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// there is a way to input drivetrain using Pedro into teleop
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop", group = "Teleop")
public class TeleOp extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx pollenshooter;
    private DcMotorEx intake1;

    @Override
    public void init() {
        // Map code variables to the names in the Driver Station Robot Configuration
        frontLeft  = hardwareMap.get(DcMotorEx.class, "fL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        backLeft   = hardwareMap.get(DcMotorEx.class, "bL");
        backRight  = hardwareMap.get(DcMotorEx.class, "bR");
        pollenshooter = hardwareMap.get(DcMotorEx.class, deviceName: "sh1");
        intake1 = hardwareMap.get(DcMotorEx.class, deviceName: "in1");
        // Example directions only: verify these on your actual robot.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_x;
        double strafe  = gamepad1.left_stick_y;
        double turn    =  gamepad1.right_stick_x;


        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Keep all four powers within the motor power range of -1 to +1
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));
        pollenshooter.setPower(0.5);
        intake1.setPower(1);
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);

    }
}
