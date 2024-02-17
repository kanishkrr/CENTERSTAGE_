package org.firstinspires.ftc.teamcode.common.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ScoringPositions {

    public static final Pose2d[] YELLOW_PIXEL_POSITIONS = {
            new Pose2d(40.2, 39, Math.toRadians(0)), //BLUE LEFT
            new Pose2d(40.2, 33, Math.toRadians(0)), //BLUE CENTER
            new Pose2d(40.7, 27, Math.toRadians(0)), //BLUE RIGHT
            new Pose2d(40.7, -27, Math.toRadians(0)), //RED LEFT
            new Pose2d(40.2, -33.8, Math.toRadians(0)), //RED CENTER
            new Pose2d(40.2, -39, Math.toRadians(0)) //RED RIGHT
    };

    public static final Pose2d[] PURPLE_PIXEL_POSITIONS = {

    };

}
