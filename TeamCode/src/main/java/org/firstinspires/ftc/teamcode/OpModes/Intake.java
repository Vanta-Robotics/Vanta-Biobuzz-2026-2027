package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake {
    private static final double INTAKE_POWER = 0.6;
    private static final long DOUBLE_CLICK_NANOS = 350_000_000L;

    private final DcMotorEx motor;
    private final Servo sorterServo;
    private static final double CLOSED_POSITION = 0;
    // TODO: Replace with the calibrated open position; both positions are currently 0.
    private static final double OPEN_POSITION = 0;
    private boolean gateLatchedOpen;
    private boolean previousGateButton;
    private long lastGatePressNanos;

    public Intake(HardwareMap hardwareMap, String motorName, String sorterServoName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        sorterServo = hardwareMap.get(Servo.class, sorterServoName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void controlIntake(boolean forwardButton, boolean reverseButton, boolean openGateButton) {
        if (forwardButton) {
            motor.setPower(INTAKE_POWER);
        } else if (reverseButton) {
            motor.setPower(-INTAKE_POWER);
        } else {
            motor.setPower(0);
        }

        // Count each press once, even when the bumper stays held across loops.
        if (openGateButton && !previousGateButton) {
            long now = System.nanoTime();
            if (lastGatePressNanos != 0 && now - lastGatePressNanos <= DOUBLE_CLICK_NANOS) {
                gateLatchedOpen = !gateLatchedOpen;
                lastGatePressNanos = 0;
            } else {
                lastGatePressNanos = now;
            }
        }
        previousGateButton = openGateButton;
        // Hold to open temporarily, or double-click to keep it open after release.
        sorterServo.setPosition(openGateButton || gateLatchedOpen ? OPEN_POSITION : CLOSED_POSITION);
    }

    public void stop() {
        motor.setPower(0);
        gateLatchedOpen = false;
        previousGateButton = false;
        lastGatePressNanos = 0;
        sorterServo.setPosition(CLOSED_POSITION);
    }
}