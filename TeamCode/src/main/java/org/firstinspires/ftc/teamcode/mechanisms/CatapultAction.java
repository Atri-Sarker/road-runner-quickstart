package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueAutonomousSimple;

public class CatapultAction {

    // Catapults
    private DcMotor leftCatapult, rightCatapult;

    // Indexer servo
    private CRServo indexServo;
    private ElapsedTime left_catatime = new ElapsedTime();
    private ElapsedTime right_catatime = new ElapsedTime();

    private ElapsedTime indextime = new ElapsedTime();

    // Powers for loading and shooting the catapult
    private double CATAPULT_UP_POWER = -1;
    private double CATAPULT_DOWN_POWER = 1;

    // INDEX LEFT POWER
    private double INDEX_LEFT_POWER = -1;
    private String LEFT_SHOOTING = "FALSE";
    private String RIGHT_SHOOTING = "FALSE";

    private String IS_INDEXING = "FALSE";

    // Delay for Catapult Sequence
    public double CatapultWaitTime = 1.5;

    // Initialize
    public CatapultAction(HardwareMap hardwareMap) {
        leftCatapult = hardwareMap.get(DcMotor.class, "left_catapult");
        rightCatapult = hardwareMap.get(DcMotor.class, "right_catapult");

        // Indexer
        indexServo = hardwareMap.get(CRServo.class, "indexServo");
        indexServo.setDirection(DcMotorSimple.Direction.FORWARD);

        rightCatapult.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        leftCatapult.setDirection(DcMotor.Direction.FORWARD);

        rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_catatime.reset();
        right_catatime.reset();
    }

    // Shooting Functions
    public class LeftShootStrong implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (LEFT_SHOOTING.equals("FALSE")) {
                LEFT_SHOOTING = "START";
                return true;
            }
            if (LEFT_SHOOTING.equals("START")) {
                left_catatime.reset();
                leftCatapult.setPower(CATAPULT_DOWN_POWER);
                LEFT_SHOOTING = "DOWN";
                return true;
            }
            if (LEFT_SHOOTING.equals("DOWN")) {
                if (left_catatime.seconds() > 0.4) {
                    LEFT_SHOOTING = "UP";
                    left_catatime.reset();
                    leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                return true;
            }
            if (LEFT_SHOOTING.equals("UP")) {
                if (left_catatime.seconds() > 0.1) {
                    leftCatapult.setPower(CATAPULT_UP_POWER);
                }
                if (left_catatime.seconds() > 0.34) {
                    leftCatapult.setPower(0);
                    LEFT_SHOOTING = "FALSE";
                    return false;
                }
            }
            return true;
        }
    }
    public Action leftShootStrong() {
        return new CatapultAction.LeftShootStrong();
    }

    public class RightShootStrong implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (RIGHT_SHOOTING.equals("FALSE")) {
                RIGHT_SHOOTING = "START";
                return true;
            }
            if (RIGHT_SHOOTING.equals("START")) {
                right_catatime.reset();
                rightCatapult.setPower(CATAPULT_DOWN_POWER);
                RIGHT_SHOOTING = "DOWN";
                return true;
            }
            if (RIGHT_SHOOTING.equals("DOWN")) {
                if (right_catatime.seconds() > 0.4) {
                    RIGHT_SHOOTING = "UP";
                    right_catatime.reset();
                    rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                return true;
            }
            if (RIGHT_SHOOTING.equals("UP")) {
                if (right_catatime.seconds() > 0.1) {
                    rightCatapult.setPower(CATAPULT_UP_POWER);
                }
                if (right_catatime.seconds() > 0.34) {
                    rightCatapult.setPower(0);
                    RIGHT_SHOOTING = "FALSE";
                    return false;
                }
            }
            return true;
        }
    }

    public Action rightShootStrong() {
        return new CatapultAction.RightShootStrong();
    }

    public class IndexToLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (IS_INDEXING.equals("FALSE")) {
                IS_INDEXING = "TRUE";
                indexServo.setPower(INDEX_LEFT_POWER);
                indextime.reset();
                return true;
            }
            if (IS_INDEXING.equals("TRUE")) {
                if (indextime.seconds() > 0.5) {
                    indexServo.setPower(0);
                    IS_INDEXING = "FALSE";
                    return false;
                }
            }
            return true;
        }
    }

    public Action indexToLeft() {
        return new CatapultAction.IndexToLeft();
    }


    public ParallelAction LeftRightLeft = new ParallelAction(
            new SequentialAction(
                    leftShootStrong(),
                    indexToLeft(),
                    new SleepAction(CatapultWaitTime),
                    leftShootStrong()
            ),
            new SequentialAction(
                    new SleepAction(CatapultWaitTime),
                    rightShootStrong()
            )
    );
}
