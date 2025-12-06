package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotFieldPosesRed {

    // DIRECTIONS
    public double directionUP = Math.toRadians(90);
    public double directionFacingRamp = Math.toRadians(0);

    public double directionFacingGoal = Math.toRadians(80);

    // Starting Positions
    public Pose2d startPoseClose = new Pose2d(-15, -65, directionUP);
    public Pose2d startPoseLong = new Pose2d(15, -65, directionUP);

    // Shooting Positions
    public  Pose2d shootPoseLong = new Pose2d(15, -60, directionFacingGoal);
    public Pose2d shootPoseMedium = new Pose2d(-48, 48, Math.toRadians(135));
    public Pose2d shootPoseClose = new Pose2d(-48, 48, Math.toRadians(135));


    // X Value for the start of artifact sets
    public double ArtifactSetStart_X = 24;
    // X Value for the end of artifact sets
    public double ArtifactSetEnd_X = 60;
    // Artifact Pickup Positions
    public Pose2d PPGStart = new Pose2d(ArtifactSetStart_X,  12, directionFacingRamp);
    public Pose2d PGPStart = new Pose2d(ArtifactSetStart_X, -12, directionFacingRamp);
    public Pose2d GPPStart = new Pose2d(ArtifactSetStart_X, -36, directionFacingRamp);

}
