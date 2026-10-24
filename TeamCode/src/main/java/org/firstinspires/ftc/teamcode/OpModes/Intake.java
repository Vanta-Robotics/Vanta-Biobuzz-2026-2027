package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake {
    private static final double INTAKE_POWER = .6;
    private static final double INTAKE_POWER_NECTOR = 1;
    private static final long DOUBLE_CLICK_NANOS = 350_000_000L;

    private final DcMotorEx pollenIntakeMotor;
    private final DcMotorEx nectarIntakeMotor;
    private final Servo rampServo;
    private static final double CLOSED_POSITION = 0;
    // TODO: Replace with the calibrated open position; both positions are currently 0.
    private static final double OPEN_POSITION = 0;
    private boolean gateLatchedOpen;
    private boolean previousGateButton;
    private long lastGatePressNanos;

    public Intake(HardwareMap hardwareMap, String pollenMotor,String nectarMotor, String servoName) {
        pollenIntakeMotor = hardwareMap.get(DcMotorEx.class, pollenMotor);
        nectarIntakeMotor = hardwareMap.get(DcMotorEx.class, nectarMotor);
        rampServo = hardwareMap.get(Servo.class, servoName);
        pollenIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void intakePollen(boolean forwardButton, boolean reverseButton, boolean openGateButton) {
        if (forwardButton) {
            pollenIntakeMotor.setPower(INTAKE_POWER);
        } else if (reverseButton) {
            pollenIntakeMotor.setPower(-INTAKE_POWER);
        } else {
            pollenIntakeMotor.setPower(0);
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
        rampServo.setPosition(openGateButton || gateLatchedOpen ? OPEN_POSITION : CLOSED_POSITION);
    }

    public void stop() {
        pollenIntakeMotor.setPower(0);
        gateLatchedOpen = false;
        previousGateButton = false;
        lastGatePressNanos = 0;
        rampServo.setPosition(CLOSED_POSITION);
    }
}
