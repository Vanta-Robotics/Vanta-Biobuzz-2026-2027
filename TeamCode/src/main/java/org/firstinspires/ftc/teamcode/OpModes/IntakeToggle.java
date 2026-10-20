package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeToggle {
    private final Servo servo;
    private final double closedPosition;
    private final double openPosition;

    public IntakeToggle(HardwareMap hardwareMap, String servoName,
                        double closedPosition, double openPosition) {
        servo = hardwareMap.get(Servo.class, servoName);
        this.closedPosition = closedPosition;
        this.openPosition = openPosition;

        // Start with the intake gate closed.
        servo.setPosition(closedPosition);
    }

    public void update(boolean openButton, boolean closeButton) {
        if (openButton) {
            servo.setPosition(openPosition);
        } else if (closeButton) {
            servo.setPosition(closedPosition);
        }
    }
}
