package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultControl;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.TDrive;

@TeleOp(name="Drive + Catapult", group="Linear OpMode")
public class OriginalTeleOp extends OpMode {

    TDrive drive = new TDrive();
    CatapultControl catapult = new CatapultControl();
    IntakeControl intakeServo = new IntakeControl();
    double throttle, spin;

    @Override
    public void init() {
        drive.init(hardwareMap);
        catapult.init(hardwareMap);
        intakeServo.init(hardwareMap);
    }

    @Override
    public void loop() {
        throttle = -Math.pow(gamepad1.left_stick_y,3);
        spin = Math.pow(gamepad1.left_stick_x, 3);

        if (gamepad1.left_bumper) {
            throttle *= 0.3;
            spin *= 0.3;
        }

        drive.drive(throttle, spin);
        catapult.handleCatapult(gamepad1.right_bumper);
        if (gamepad1.a) {
            intakeServo.drive(1.0);
        } else if (gamepad1.b) {
            intakeServo.drive(-1.0);
        } else {
            intakeServo.drive(0.0);
        }
    }

}