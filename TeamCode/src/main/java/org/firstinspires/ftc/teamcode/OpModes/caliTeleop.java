package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Calibration Teleop", group = "TeleOp")
public class caliTeleop extends OpMode{

    private Servo blocker;

    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "blocker");
    }
    @Override
    public void loop(){
        if (gamepad1.a){
            blocker.setPosition(0);
        }
    }
}
