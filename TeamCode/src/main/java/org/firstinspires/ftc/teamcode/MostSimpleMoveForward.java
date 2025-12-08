package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesRed;
import org.firstinspires.ftc.teamcode.mechanisms.WebcamControl;

@Config
@Autonomous(name = "RED AUTO [SCAN CAM] [SHOOT ONCE]", group = "Autonomous")
public class MostSimpleMoveForward extends LinearOpMode  {
    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesRed Field = new RobotFieldPosesRed();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT WEBCAM SYSTEM
        WebcamControl CameraScanner = new WebcamControl(hardwareMap);

        // Start at a square facing the motif
        Pose2d initialPose = Field.startPoseClose;
        // Initialize Drive
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // SCAN WEBCAM
        String scannedMotif = CameraScanner.waitForDecodeMotif(5.0);

        telemetry.addData("TAG", scannedMotif);
        telemetry.update();
        // DRIVE TO SOEMEWHERe
        TrajectoryActionBuilder tabStartPoseToShootPose = drive.actionBuilder(initialPose).
                splineTo(Field.shootPoseClose.position, Field.shootPoseClose.heading);
        Actions.runBlocking(
                tabStartPoseToShootPose.build()
        );


    }
}
