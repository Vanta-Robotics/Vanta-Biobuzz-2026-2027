package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake {
    private static final double INTAKE_POWER = 0.6;

    private final DcMotorEx motor;
    private final Servo servo;
    private static final double CLOSED_POSITION = 0;
    // TODO: Replace with the calibrated open position; both positions are currently 0.
    private static final double OPEN_POSITION = 0;

    public Intake(HardwareMap hardwareMap, String motorName, String servoName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        servo = hardwareMap.get(Servo.class, servoName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void update(boolean forwardButton, boolean reverseButton, boolean openGateButton) {
        if (forwardButton) {
            motor.setPower(INTAKE_POWER);
        } else if (reverseButton) {
            motor.setPower(-INTAKE_POWER);
        } else {
            motor.setPower(0);
        }

        // Hold the button to open the gate; releasing it closes the gate.
        servo.setPosition(openGateButton ? OPEN_POSITION : CLOSED_POSITION);
    }

    public void stop() {
        motor.setPower(0);
        servo.setPosition(CLOSED_POSITION);
    }
}
