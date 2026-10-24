package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Shooter {
    // Starting gains, not robot-tuned values. Error is a fraction of full motor speed.
    public static volatile double KP = 0.2;
    public static volatile double KI = 0.0;
    public static volatile double KD = 0.0;
    public static volatile double KF = 1.0;
    private static final long DOUBLE_CLICK_NANOS = 350_000_000L;

    private final DcMotorEx motor;
    private final Servo blocker;
    private final double closedPosition;
    private final double openPosition;
    private final double fullSpeed;
    private final double maxVelocityTicksPerSecond;
    private double targetVelocityTicksPerSecond;
    private double outputPower;
    private double integral;
    private double previousSpeed;
    private long lastUpdateNanos;
    private boolean blockerLatchedOpen;
    private boolean previousBlockerButton;
    private long lastBlockerPressNanos;

    public Shooter(HardwareMap hardwareMap, String motorName, double maxSpeedFraction,
                   String blockerName, double closedPosition, double openPosition) {
        if (!Double.isFinite(maxSpeedFraction) || maxSpeedFraction <= 0 || maxSpeedFraction > 1) {
            throw new IllegalArgumentException("Shooter max speed fraction must be greater than 0 and at most 1");
        }
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        // Speeds come from the motor model selected in the robot configuration.
        fullSpeed = motor.getMotorType().getAchieveableMaxTicksPerSecond();
        maxVelocityTicksPerSecond = fullSpeed * maxSpeedFraction;
        if (!Double.isFinite(maxVelocityTicksPerSecond) || maxVelocityTicksPerSecond <= 0) {
            throw new IllegalArgumentException("Shooter needs a motor configuration with a valid maximum speed");
        }

        // Read the encoder, but let our PID control setPower instead of the hub's PID.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();

        blocker = hardwareMap.get(Servo.class, blockerName);
        this.closedPosition = closedPosition;
        this.openPosition = openPosition;
        blocker.setPosition(closedPosition);
    }

    public void updateBlocker(boolean button) {
        // Count a new press once, rather than counting every loop while held.
        if (button && !previousBlockerButton) {
            long now = System.nanoTime();
            if (lastBlockerPressNanos != 0 && now - lastBlockerPressNanos <= DOUBLE_CLICK_NANOS) {
                blockerLatchedOpen = !blockerLatchedOpen;
                lastBlockerPressNanos = 0;
            } else {
                lastBlockerPressNanos = now;
            }
        }
        previousBlockerButton = button;
        // A hold opens temporarily; a double-click keeps it open after release.
        blocker.setPosition(button || blockerLatchedOpen ? openPosition : closedPosition);
    }

    public void update(double trigger) {
        if (!Double.isFinite(trigger) || trigger <= 0) {
            stop();
            return;
        }
        double demand = Math.max(0, Math.min(1, trigger));
        targetVelocityTicksPerSecond = demand * maxVelocityTicksPerSecond;
        double speed = motor.getVelocity() / fullSpeed;
        if (!Double.isFinite(speed)) {
            stop();
            return;
        }
        double target = targetVelocityTicksPerSecond / fullSpeed;
        double error = target - speed;

        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0 ? 0 : (now - lastUpdateNanos) / 1_000_000_000.0;
        double nextIntegral = integral;
        double derivative = 0;
        // Skip I and D on the first update or after a pause longer than 250 ms.
        if (dt > 0 && dt <= 0.25) {
            if (KI != 0) nextIntegral += error * dt;
            // Use speed change so moving the trigger doesn't create a D spike.
            derivative = -(speed - previousSpeed) / dt;
        }

        double basePower = KF * target + KP * error + KD * derivative;
        double requestedPower = basePower + KI * nextIntegral;
        // Don't build up more integral when power is already at its limit.
        if (!((requestedPower > 1 && error > 0) || (requestedPower < 0 && error < 0))) {
            integral = nextIntegral;
        }
        outputPower = Math.max(0, Math.min(1, basePower + KI * integral));
        motor.setPower(outputPower);
        previousSpeed = speed;
        lastUpdateNanos = now;
    }

    public double getTargetRpm() {
        return targetVelocityTicksPerSecond * 60.0 / motor.getMotorType().getTicksPerRev();
    }

    public double getActualRpm() {
        return motor.getVelocity() * 60.0 / motor.getMotorType().getTicksPerRev();
    }

    public double getOutputPower() {
        return outputPower;
    }

    public void stop() {
        targetVelocityTicksPerSecond = 0;
        outputPower = 0;
        integral = 0;
        previousSpeed = 0;
        lastUpdateNanos = 0;
        motor.setPower(0);
    }
}
