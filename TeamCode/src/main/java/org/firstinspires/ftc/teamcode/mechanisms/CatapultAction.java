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
    public double CATAPULT_UP_POWER = -1;
    private double CATAPULT_DOWN_POWER = 1;

    // INDEX LEFT POWER
    private double INDEX_LEFT_POWER = -0.25;
    private String LEFT_SHOOTING = "FALSE";
    private String RIGHT_SHOOTING = "FALSE";

    private String IS_INDEXING = "FALSE";

    // Delay for Catapult Sequence
    public double CatapultWaitTime = 0.5;

    // Launch
    private double CatapultDownTime = 0.5;
    private double CatapultHoldTime = 0.2;
    private double CatapultReleaseTime = 0.32;

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
                if (left_catatime.seconds() > CatapultDownTime) {
                    LEFT_SHOOTING = "UP";
                    left_catatime.reset();
                    leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                return true;
            }
            if (LEFT_SHOOTING.equals("UP")) {
                if (left_catatime.seconds() > CatapultHoldTime) {
                    leftCatapult.setPower(CATAPULT_UP_POWER);
                }
                if (left_catatime.seconds() > CatapultReleaseTime) {
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
                if (right_catatime.seconds() > CatapultDownTime) {
                    RIGHT_SHOOTING = "UP";
                    right_catatime.reset();
                    rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                return true;
            }
            if (RIGHT_SHOOTING.equals("UP")) {
                if (right_catatime.seconds() > CatapultHoldTime) {
                    rightCatapult.setPower(CATAPULT_UP_POWER);
                }
                if (right_catatime.seconds() > CatapultReleaseTime) {
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
            indexServo.setPower(INDEX_LEFT_POWER);
            if (IS_INDEXING.equals("FALSE")) {
                IS_INDEXING = "TRUE";
                indexServo.setPower(INDEX_LEFT_POWER);
                indextime.reset();
                return true;
            }
            if (IS_INDEXING.equals("TRUE")) {
                if (indextime.seconds() > 2.0) {
                    indexServo.setPower(0);
                    IS_INDEXING = "FALSE";
                    return false;
                }
            } else {
                indexServo.setPower(INDEX_LEFT_POWER);
            }
            return true;
        }
    }

    public Action indexToLeft() {
        return new CatapultAction.IndexToLeft();
    }

    public class IndexToRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            indexServo.setPower(-1*INDEX_LEFT_POWER);
            if (IS_INDEXING.equals("FALSE")) {
                IS_INDEXING = "TRUE";
                indexServo.setPower(-1*INDEX_LEFT_POWER);
                indextime.reset();
                return true;
            }
            if (IS_INDEXING.equals("TRUE")) {
                if (indextime.seconds() > 2.0) {
                    indexServo.setPower(0);
                    IS_INDEXING = "FALSE";
                    return false;
                } else {
                    indexServo.setPower(-1*INDEX_LEFT_POWER);
                }
            }
            return true;
        }
    }

    public Action indexToRight() {
        return new CatapultAction.IndexToRight();
    }


    public ParallelAction LeftRightLeft() {
        return new ParallelAction(
            new SequentialAction(
                    leftShootStrong(),
                    indexToLeft(),
                    new SleepAction(CatapultWaitTime*0.2),
                    leftShootStrong()
            ),
            new SequentialAction(
                    new SleepAction(CatapultWaitTime),
                    rightShootStrong()
            )
        );
    }

    public ParallelAction RightLeftRight() {
        return new ParallelAction(
                new SequentialAction(
                        rightShootStrong(),
                        indexToRight(),
                        new SleepAction(CatapultWaitTime*0.2),
                        rightShootStrong()
                ),
                new SequentialAction(
                        new SleepAction(CatapultWaitTime),
                        leftShootStrong()
                )
        );
    }

    public ParallelAction LeftLeftRight() {
        return new ParallelAction(
                new SequentialAction(
                        leftShootStrong(),
                        indexToLeft(),
                        new SleepAction(CatapultWaitTime*0.2),
                        leftShootStrong(),
                        new SleepAction(CatapultWaitTime*0.2),
                        rightShootStrong()
                ),
                new SleepAction(CatapultWaitTime)
        );
    }

    public ParallelAction RightRightLeft() {
        return new ParallelAction(
                new SequentialAction(
                        rightShootStrong(),
                        indexToRight(),
                        new SleepAction(CatapultWaitTime*0.2),
                        rightShootStrong(),
                        new SleepAction(CatapultWaitTime*0.2),
                        leftShootStrong()
                ),
                new SleepAction(CatapultWaitTime)
        );
    }

    public ParallelAction getShootAction(String targetMotif, String currentBarrel) {
        if (targetMotif.equals("PGP")) {
            if (currentBarrel.equals("PPG")) {
                return LeftRightLeft();
            }
            if (currentBarrel.equals("PGP")) {
                return LeftLeftRight();
            }
            if (currentBarrel.equals("GPP")) {
                return RightLeftRight();
            }
        }
        if (targetMotif.equals("PPG")) {
            if (currentBarrel.equals("PPG")) {
                return LeftLeftRight();
            }
            if (currentBarrel.equals("PGP")) {
                return LeftRightLeft();
            }
            if (currentBarrel.equals("GPP")) {
                return RightRightLeft();
            }
        }
        if (targetMotif.equals("GPP")) {
            if (currentBarrel.equals("PPG")) {
                return RightLeftRight();
            }
            if (currentBarrel.equals("PGP")) {
                // FAIL
                return LeftLeftRight();
            }
            if (currentBarrel.equals("GPP")) {
                return LeftRightLeft();
            }
        }

        // Random pattern
        return LeftLeftRight();
    }
}
