package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake {
    private static final double INTAKE_POWER = 0.6;

    private final DcMotorEx motor;
    private final Servo sorterServo;

    public Intake(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
    }

    public void controlIntake(boolean forwardButton, boolean reverseButton) {
        if (forwardButton) {
            motor.setPower(INTAKE_POWER);
        } else if (reverseButton) {
            motor.setPower(-INTAKE_POWER);
        } else {
            stop();
        }
    }

    public void controlIntake(boolean forwardButton, boolean reverseButton, boolean sortButton) {
        controlIntake(forwardButton, reverseButton);

        if (sortButton) {
            sorterServo.setPosition(1.0);
        } else {
            sorterServo.setPosition(0.0);
        }
    }

    public void stop() {
        motor.setPower(0);
    }
}