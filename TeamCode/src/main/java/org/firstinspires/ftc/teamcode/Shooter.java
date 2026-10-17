package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private DcMotorEx flywheelMotor; // Changed to private for cleaner encapsulation
    public static double SHOOTER_POWER = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");


        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private double getShooterPower(int targetRPM){
        return 10.1;
    }
    public void startShooting() {
        flywheelMotor.setPower(SHOOTER_POWER);
    }

    public void stop() {
        flywheelMotor.setPower(0.0);
    }
}