package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.mechanisms.CatapultControl;

@Config
@Autonomous(name = "Red Autonomous Simple", group = "Autonomous")
public class RedAutonomousSimple extends LinearOpMode {
    public class Intake {
        private CRServo firstRoller;
        private CRServo secondRoller;

        public Intake(HardwareMap hardwareMap) {
            firstRoller = hardwareMap.get(CRServo.class, "bottom");
            secondRoller = hardwareMap.get(CRServo.class, "top");
            firstRoller.setDirection(DcMotorSimple.Direction.FORWARD);
            secondRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class PullArtifacts implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                firstRoller.setPower(1.0);
                secondRoller.setPower(1.0);
                return false;
            }
        }
        public Action pullArtifacts() {
            return new PullArtifacts();
        }

        public class PushArtifacts implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                firstRoller.setPower(-1.0);
                secondRoller.setPower(-1.0);
                return false;
            }
        }
        public Action pushArtifacts() {
            return new PushArtifacts();
        }

        public class StopRolling implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                firstRoller.setPower(0);
                secondRoller.setPower(0);
                return false;
            }
        }
        public Action stopRolling() {
            return new StopRolling();
        }

    }

    public class Catapult {
        private DcMotor leftCatapult, rightCatapult;
        private ElapsedTime catatime = new ElapsedTime();
        private double CATAPULT_UP_POWER = -1;
        private double CATAPULT_DOWN_POWER = 1;
        private String SHOOTING = "FALSE";
        public Catapult(HardwareMap hardwareMap) {
            leftCatapult = hardwareMap.get(DcMotor.class, "left_catapult");
            rightCatapult = hardwareMap.get(DcMotor.class, "right_catapult");

            rightCatapult.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
            leftCatapult.setDirection(DcMotor.Direction.FORWARD);

            rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            catatime.reset();
        }

        public class Shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (SHOOTING.equals("FALSE")) {
                    SHOOTING = "START";
                    return true;
                }
                if (SHOOTING.equals("START")) {
                    catatime.reset();
                    leftCatapult.setPower(1);
                    rightCatapult.setPower(1);
                    SHOOTING = "DOWN";
                    return true;
                }
                if (SHOOTING.equals("DOWN")) {
                    if (catatime.seconds() > 0.4) {
                        SHOOTING = "UP";
                        catatime.reset();
                        rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    return true;
                }
                if (SHOOTING.equals("UP")) {
                    if (catatime.seconds() > 0.1) {
                        leftCatapult.setPower(-1);
                        rightCatapult.setPower(-1);
                    }
                    if (catatime.seconds() > 0.34) {
                        leftCatapult.setPower(0);
                        rightCatapult.setPower(0);
                        SHOOTING = "FALSE";
                        return false;
                    }
                }
                return true;
            }
        }
        public Action shoot() {
            return new Catapult.Shoot();
        }

    }
    @Override
    public void runOpMode() {
        // instantiate your TankDrive at a particular pose.
        Pose2d initialPose = new Pose2d(48, 48, Math.toRadians(45));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        // Intake
        Intake intake = new Intake(hardwareMap);
        // Catapult
        Catapult catapult = new Catapult(hardwareMap);

//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToX(-38)
//                .waitSeconds(0.3);
//
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToX(-16)
//                .waitSeconds(0.5)
//                .turnTo(Math.toRadians(180))
//                .waitSeconds(0.5);
//
//        TrajectoryActionBuilder tab2b = drive.actionBuilder(initialPose)
//                .lineToX(-16)
//                .turnTo(Math.toRadians(180));
//
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToX(-55).waitSeconds(2).lineToX(-16).turnTo(Math.toRadians(135));
//
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
//                .turnTo(Math.toRadians(90)).lineToY(-18);

        TrajectoryActionBuilder goToShootPos = drive.actionBuilder(initialPose)
                .turnTo(Math.toRadians(45))
                .lineToX(38)
                .waitSeconds(0.1);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToX(12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder preparePick = drive.actionBuilder(initialPose)
                .turnTo(Math.toRadians(0))
                .waitSeconds(0.1);

        TrajectoryActionBuilder forwardAndBack = drive.actionBuilder(initialPose)
                .lineToX(50)
                .waitSeconds(0.3)
                .lineToX(12).waitSeconds(0.1);

        TrajectoryActionBuilder goDown1 = drive.actionBuilder(initialPose)
                .turnTo((Math.toRadians(90)))
                .lineToY(-12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder goUp = drive.actionBuilder(initialPose)
                .turnTo((Math.toRadians(90)))
                .lineToY(12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder goDown2 = drive.actionBuilder(initialPose)
                .turnTo((Math.toRadians(90)))
                .lineToY(-36)
                .waitSeconds(0.1);


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        goToShootPos.build(),
                        catapult.shoot(),
                        tab2.build(),
                        preparePick.build(),
                        intake.pullArtifacts(),
                        forwardAndBack.build(),
                        intake.stopRolling(),
                        goToShootPos.build(),
                        catapult.shoot(),
                        tab2.build(),
                        goDown1.build(),
                        preparePick.build(),
                        intake.pullArtifacts(),
                        forwardAndBack.build(),
                        intake.stopRolling(),
                        goUp.build(),
                        goToShootPos.build(),
                        catapult.shoot(),
                        tab2.build(),
                        goDown2.build(),
                        preparePick.build(),
                        intake.pullArtifacts(),
                        forwardAndBack.build(),
                        intake.stopRolling()
                )
        );

    }
}
