package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TeleOp", group = "TeleOp")
// extends opmode wasn't there last time
public class teleOp extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private Intake intakeA;
    private ShooterStopper shooterStopper;

    @Override
    public void init() {

        intakeA = new Intake(hardwareMap, "in1"); //check name
        shooterStopper = new ShooterStopper(hardwareMap,
                "stopperA", 0, 0.5); //adjust name and value
        //first value is close position second is open position of servo

        // Map code variables to the names in the Driver Station Robot Configuration
        frontLeft  = hardwareMap.get(DcMotorEx.class, "fL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        backLeft   = hardwareMap.get(DcMotorEx.class, "bL");
        backRight  = hardwareMap.get(DcMotorEx.class, "bR");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

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

        // a runs intake forward, b reverses it
        intakeA.update(gamepad1.a, gamepad1.b);

        shooterStopper.update(gamepad1.x, gamepad1.y);

        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        // this makes up for stick drift (maybe) change values if necessary
        if (Math.abs(forward) < 0.08) forward = 0;
        if (Math.abs(strafe) < 0.08) strafe = 0;
        if (Math.abs(turn) < 0.08) turn = 0;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Keep all four powers within the motor power range of -1 to +1
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);



    }
}