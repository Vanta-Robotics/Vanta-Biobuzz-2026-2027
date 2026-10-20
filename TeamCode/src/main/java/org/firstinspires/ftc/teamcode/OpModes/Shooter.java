package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Shooter {
    private final DcMotorEx motor;
    private final double maxPower;

    public Shooter(HardwareMap hardwareMap, String motorName, double maxPower) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.maxPower = maxPower;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void update(double trigger) {
        // The trigger controls how much of the shooter's maximum power to use.
        motor.setPower(trigger * maxPower);
    }

    public void stop() {
        motor.setPower(0);
    }
}
