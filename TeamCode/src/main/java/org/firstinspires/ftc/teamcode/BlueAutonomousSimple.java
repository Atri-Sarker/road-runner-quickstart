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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@Autonomous(name = "Blue Autonomous Simple", group = "Autonomous")
public class BlueAutonomousSimple extends LinearOpMode {
    public class Intake {
        private CRServo firstRoller;
        private CRServo secondRoller;

        private CRServo finalRoller;

        public Intake(HardwareMap hardwareMap) {
            firstRoller = hardwareMap.get(CRServo.class, "bottom");
            secondRoller = hardwareMap.get(CRServo.class, "top");
            finalRoller = hardwareMap.get(CRServo.class, "final");
            firstRoller.setDirection(DcMotorSimple.Direction.FORWARD);
            secondRoller.setDirection(DcMotorSimple.Direction.REVERSE);
            finalRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class PullArtifacts implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                firstRoller.setPower(1.0);
                secondRoller.setPower(1.0);
                finalRoller.setPower(1.0);
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
                finalRoller.setPower(-1.0);
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
                finalRoller.setPower(0);
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
        Pose2d initialPose = new Pose2d(-48, 48, Math.toRadians(135));
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

        TrajectoryActionBuilder goToShootPos1 = drive.actionBuilder(initialPose).fresh()
                .turnTo(Math.toRadians(130))
              .lineToX(-38)
              .waitSeconds(0.1)
                .turnTo(Math.toRadians(135));

        TrajectoryActionBuilder middleman1 = goToShootPos1.endTrajectory().fresh()
                .lineToX(-12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder preparePick1 = middleman1.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .waitSeconds(0.1);

        TrajectoryActionBuilder forwardAndBack1 = preparePick1.endTrajectory().fresh()
                .lineToX(-50)
                .waitSeconds(0.3)
                .lineToX(-12).waitSeconds(0.1);

        TrajectoryActionBuilder goToShootPos2 = forwardAndBack1.endTrajectory().fresh()
                .turnTo(Math.toRadians(130))
                .lineToX(-38)
                .waitSeconds(0.1).turnTo(Math.toRadians(135));

        TrajectoryActionBuilder middleman2 = goToShootPos2.endTrajectory().fresh()
                .lineToX(-12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder goDown1 = middleman2.endTrajectory().fresh()
                    .turnTo((Math.toRadians(90)))
                    .lineToY(-12)
                    .waitSeconds(0.1);

        TrajectoryActionBuilder preparePick2 = goDown1.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .waitSeconds(0.1);

        TrajectoryActionBuilder forwardAndBack2 = preparePick2.endTrajectory().fresh()
                .lineToX(-50)
                .waitSeconds(0.3)
                .lineToX(-12).waitSeconds(0.1);

        TrajectoryActionBuilder goUp = forwardAndBack2.endTrajectory().fresh()
                .turnTo((Math.toRadians(90)))
                .lineToY(12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder goToShootPos3 = goUp.endTrajectory().fresh()
                .turnTo(Math.toRadians(130))
                .lineToX(-38)
                .waitSeconds(0.1).turnTo(Math.toRadians(135));

        TrajectoryActionBuilder middleman3 = goToShootPos3.endTrajectory().fresh()
                .lineToX(-12)
                .waitSeconds(0.1);

        TrajectoryActionBuilder goDown2 = middleman3.endTrajectory().fresh()
                .turnTo((Math.toRadians(90)))
                .lineToY(-36)
                .waitSeconds(0.1);

        TrajectoryActionBuilder preparePick3 = goDown2.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .waitSeconds(0.1);

        TrajectoryActionBuilder forwardAndBack3 = preparePick3.endTrajectory().fresh()
                .lineToX(-50)
                .waitSeconds(0.3)
                .lineToX(-12).waitSeconds(0.1);


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        goToShootPos1.build(),
                        catapult.shoot(),
                        middleman1.build(),
                        preparePick1.build(),
                        intake.pullArtifacts(),
                        forwardAndBack1.build(),
                        intake.stopRolling(),
                        goToShootPos2.build(),
                        catapult.shoot(),
                        middleman2.build(),
                        goDown1.build(),
                        preparePick2.build(),
                        intake.pullArtifacts(),
                        forwardAndBack2.build(),
                        intake.stopRolling(),
                        goUp.build(),
                        goToShootPos3.build(),
                        catapult.shoot(),
                        middleman3.build(),
                        goDown2.build(),
                        preparePick3.build(),
                        intake.pullArtifacts(),
                        forwardAndBack3.build(),
                        intake.stopRolling()
                )
        );

    }
}
