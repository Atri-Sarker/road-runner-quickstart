package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IndexServoAction;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesBlue;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesRed;

@Config
@Autonomous(name = "Red Autonomous Close Side [No Webcam] [NO SYSTEMS]", group = "Autonomous")
public class RedAutoCloseNoWebcamNoSystem extends LinearOpMode  {
    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesRed Field = new RobotFieldPosesRed();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT INDEXER ACTIONS
        IndexServoAction IndexerSystem = new IndexServoAction(hardwareMap);
        // Start Position
        Pose2d initialPose = Field.shootPoseClose;
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // SCAN WEBCAM [NO]
        Actions.runBlocking(new SleepAction(0.5));

        // Shoot
        Actions.runBlocking(CatapultSystem.LeftRightLeft);

    }
}
