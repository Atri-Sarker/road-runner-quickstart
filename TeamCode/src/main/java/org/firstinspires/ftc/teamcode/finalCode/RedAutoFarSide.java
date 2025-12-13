package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeAction;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesRed;
import org.firstinspires.ftc.teamcode.mechanisms.WebcamControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Autonomous(name = "RED AUTO [FAR SIDE] [WITH WEBCAM]", group = "Autonomous")
public class RedAutoFarSide extends LinearOpMode  {

    public WebcamControl CameraScanner;
    public String scannedMotif = "PGP";
    public ElapsedTime scantime = new ElapsedTime();
    // FUNCTION FOR SCANNING WEBCAM
    public class scanAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (scantime.seconds() < 4.0) {
                List<AprilTagDetection> currentDetections = CameraScanner.aprilTag.getDetections();

                for (AprilTagDetection tag : currentDetections) {

                    String motif = CameraScanner.decodeMotifFromID(tag.id);

                    if (!motif.equals("UNKNOWN")) {
                        // SET motif to found motif
                        scannedMotif = motif;
                        // Disable April Tag Processor
                        CameraScanner.portal.setProcessorEnabled(CameraScanner.aprilTag, false);
                        return false;     // return immediately when a valid motif tag is found
                    }
                }
                return true;
            }
            return false;
        }
    }

    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesRed Field = new RobotFieldPosesRed();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT WEBCAM SYSTEM
        CameraScanner = new WebcamControl(hardwareMap);
        // IMPORT INTAKE ACTIONS
        IntakeAction IntakeSystem = new IntakeAction(hardwareMap);

        // Start facing the goal
        Pose2d initialPose = Field.initialFarPose;
        // Initialize Drive
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // CATAPULT POWER SET TO A LOWER NUMBER
        CatapultSystem.CATAPULT_UP_POWER = -0.9;

        // WAIT FOR START
        waitForStart();
        scantime.reset();

        // GO TO STARTING POSE
        TrajectoryActionBuilder tabStartPoseToStartPose = drive.actionBuilder(initialPose)
                .lineToY(Field.startPoseClose.position.y);
        Actions.runBlocking(
                new ParallelAction(
                        tabStartPoseToStartPose.build(),
                        new scanAction()
                )
        );

        // SCAN WEBCAM

        telemetry.addData("TAG", scannedMotif);
        telemetry.update();
        // DRIVE TO SHOOTING POSITION
        TrajectoryActionBuilder tabStartPoseToShootPose = tabStartPoseToStartPose.endTrajectory().fresh()
                .turnTo(Field.directionFacingGoal);
        Actions.runBlocking(
                tabStartPoseToShootPose.build()
        );

        // Shoot [First Load is always PPG]
        Actions.runBlocking(CatapultSystem.getShootAction(scannedMotif, "PPG"));

        // LET IMU REST
        sleep(500);

        // Rotate towards UP and go down
        TrajectoryActionBuilder shootPoseToFirstSet = drive.actionBuilder(Field.shootPoseClose)
                .turnTo(Field.directionUP)
                .lineToY(Field.PGPStart.position.y)
                .turnTo(Field.directionFacingRamp);
        Actions.runBlocking(shootPoseToFirstSet.build());


        // Pick Up Artifacts and Go Back [First Set]
        TrajectoryActionBuilder pickTab1 = shootPoseToFirstSet.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack1 = pickTab1.endTrajectory().fresh()
                .lineToX(Field.PGPStart.position.x)
                .turnTo(Field.directionUP)
                .lineToY(Field.shootPoseClose.position.y)
                .turnTo(Field.directionFacingGoal);
        // Intake Action
        Action intakeIndex = IntakeSystem.intakeForSeconds(4.0);

        //Actions.runBlocking(new ParallelAction(intakeIndex, new SequentialAction(pickTab1.build(), goBack1.build())));
    }
}
