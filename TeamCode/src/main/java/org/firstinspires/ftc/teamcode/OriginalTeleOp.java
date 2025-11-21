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
        throttle = -gamepad1.left_stick_y;
        spin = gamepad1.left_stick_x;

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