package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.CatapultControl;
import org.firstinspires.ftc.teamcode.mechanisms.IndexServoAction;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.TDrive;

@TeleOp(name="Drive + Catapult", group="Linear OpMode")
@Disabled
public class OriginalTeleOp extends OpMode {

    TDrive drive = new TDrive();
    CatapultAction catapultSystem;
    IntakeControl intakeServo = new IntakeControl();

    IndexServoAction indexer;
    double throttle, spin;
    boolean intakeToggled = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        catapultSystem = new CatapultAction(hardwareMap);
        intakeServo.init(hardwareMap);
        indexer = new IndexServoAction(hardwareMap);
    }

    @Override
    public void loop() {
        throttle = -Math.pow(gamepad1.left_stick_y,3);
        spin = Math.pow(gamepad1.left_stick_x, 3);

        if (gamepad1.dpad_left) {
            spin = -0.6;
        }
        if (gamepad1.dpad_right) {
            spin = 0.6;
        }
        if (gamepad1.dpad_down) {
            throttle = -0.6;
        }
        if (gamepad1.dpad_up) {
            throttle = 0.6;
        }

        if (gamepad1.left_trigger >= 0.5) {
            throttle *= 0.3;
            spin *= 0.3;
        }

        drive.drive(throttle, spin);
        if (gamepad1.left_bumper) {
            Actions.runBlocking(catapultSystem.leftShootStrong());
        }
        if (gamepad1.right_bumper) {
            Actions.runBlocking(catapultSystem.rightShootStrong());
        }
        if (gamepad1.right_trigger >= 0.5) {
            Actions.runBlocking(catapultSystem.LeftRightLeft());
        }
        if (gamepad1.x) {
            indexer.ultraLeft();
        }
        else if (gamepad1.y) {
            indexer.ultraRight();
        } else {
            indexer.stop();
        }
        if (gamepad1.a) {
            intakeServo.drive(1.0);
            intakeToggled = true;
        } else if (gamepad1.b) {
            intakeServo.drive(-0.7);
            intakeToggled = true;
        } else {
            intakeServo.drive(0.0);
        }
    }

}