package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TDrive {
    private DcMotor leftMotor, rightMotor;

    double currentThrottlePower = 0.0;
    double currentSpinPower = 0.0;

    final double ACCEL_LIMIT = 0.05;

    public void init(HardwareMap hwmap) {
        leftMotor = hwmap.get(DcMotor.class, "left_motor");
        rightMotor = hwmap.get(DcMotor.class, "right_motor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double desiredThrottle, double desiredSpin) {
        // Ramp the throttle input
        if (Math.abs(desiredThrottle - currentThrottlePower) > ACCEL_LIMIT) {
            currentThrottlePower += Math.signum(desiredThrottle - currentThrottlePower) * ACCEL_LIMIT;
        } else {
            currentThrottlePower = desiredThrottle;
        }

        // Ramp the spin input
        if (Math.abs(desiredSpin - currentSpinPower) > ACCEL_LIMIT) {
            currentSpinPower += Math.signum(desiredSpin - currentSpinPower) * ACCEL_LIMIT;
        } else {
            currentSpinPower = desiredSpin;
        }

        // Apply the ramped power to the motors
        double leftPower = currentThrottlePower + currentSpinPower;
        double rightPower = currentThrottlePower - currentSpinPower;
        double largest = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (largest > 1.0) {
            leftPower /= largest;
            rightPower /= largest;
        }

        // Set motor powers, ensuring values are within the -1 to 1 range
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}