package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter Test TeleOp", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private Shooter shooter;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);

        telemetry.addData("Status", "Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                shooter.startShooting();
                telemetry.addData("Shooter", "RUNNING");
            } else {
                shooter.stop();
                telemetry.addData("Shooter", "STOPPED");
            }

            telemetry.update();
        }
    }
}