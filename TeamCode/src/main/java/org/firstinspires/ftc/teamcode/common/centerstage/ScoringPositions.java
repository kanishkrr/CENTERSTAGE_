package org.firstinspires.ftc.teamcode.common.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ScoringPositions {

    public static final Pose2d[] YELLOW_PIXEL_POSITIONS = {
            new Pose2d(41, 38, Math.toRadians(0)), //BLUE LEFT
            new Pose2d(41, 32, Math.toRadians(0)), //BLUE CENTER
            new Pose2d(41, 26, Math.toRadians(0)), //BLUE RIGHT
            new Pose2d(41, -26, Math.toRadians(0)), //RED LEFT
            new Pose2d(41, -32, Math.toRadians(0)), //RED CENTER
            new Pose2d(41, -38, Math.toRadians(0)) //RED RIGHT
    };

    public static final Pose2d[] PURPLE_PIXEL_POSITIONS = {

    };

}
