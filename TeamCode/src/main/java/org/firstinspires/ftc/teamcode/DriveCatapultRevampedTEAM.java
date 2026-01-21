package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IndexServoAction;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.TDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Drive + Catapult [MULTIPLE]", group="Linear OpMode")
public class DriveCatapultRevampedTEAM extends OpMode {

    TDrive drive = new TDrive();
    CatapultAction catapultSystem;
    IntakeControl intakeServo = new IntakeControl();

    IndexServoAction indexer;
    double throttle, spin;
    boolean intakeToggled = false;

    boolean catapultDebounce = true;

    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        drive.init(hardwareMap);
        catapultSystem = new CatapultAction(hardwareMap);
        intakeServo.init(hardwareMap);
        indexer = new IndexServoAction(hardwareMap);
    }

    @Override
    public void loop() {

        // MOVEMENT
        throttle = -Math.pow(gamepad1.left_stick_y,3);
        spin = Math.pow(gamepad1.right_stick_x, 3);

        if (gamepad1.dpad_left) {
            spin = -0.3;
        }
        if (gamepad1.dpad_right) {
            spin = 0.3;
        }
        if (gamepad1.dpad_down) {
            throttle = -0.3;
        }
        if (gamepad1.dpad_up) {
            throttle = 0.3;
        }

        if (gamepad1.left_trigger >= 0.5) {
            throttle *= 0.3;
            spin *= 0.3;
        }

        drive.drive(throttle, spin);

        // HANDLE ACTIONS
        TelemetryPacket packet = new TelemetryPacket();
        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            } else {
                catapultDebounce = true;
            }
        }
        runningActions = newActions;
        
        if (gamepad2.left_bumper && catapultDebounce) {
            runningActions.add(catapultSystem.leftShootStrong());
            catapultDebounce = false;
        }
        if (gamepad2.right_bumper && catapultDebounce) {
            runningActions.add(catapultSystem.rightShootStrong());
            catapultDebounce = false;
        }
        if (gamepad2.right_trigger >= 0.5 && catapultDebounce) {
            runningActions.add(catapultSystem.LeftRightLeft());
            catapultDebounce = false;
        }
        if (gamepad2.x) {
            indexer.ultraLeft();
        }
        else if (gamepad2.y) {
            indexer.ultraRight();
        } else {
            indexer.stop();
        }
        if (gamepad2.a) {
            intakeServo.drive(1.0);
            intakeToggled = true;
        } else if (gamepad2.b) {
            intakeServo.drive(-0.8);
            intakeToggled = true;
        } else {
            intakeServo.drive(0.0);
        }
    }

}