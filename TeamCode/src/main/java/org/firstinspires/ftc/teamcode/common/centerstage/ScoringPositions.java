package org.firstinspires.ftc.teamcode.common.centerstage;


import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public class ScoringPositions {

    public static final Pose[] YELLOW_PIXEL_POSITIONS = {
            //close side
            new Pose(-26, 21, Math.toRadians(90)), //BLUE LEFT
            new Pose(-26, 37, Math.toRadians(90)), //BLUE CENTER
            new Pose(-26, 33, Math.toRadians(90)), //BLUE RIGHT
            new Pose(26, 33.3, Math.toRadians(-90)), //RED LEFT
            new Pose(25.7, 28, Math.toRadians(-90)), //RED CENTER
            new Pose(25.7, 22, Math.toRadians(-90)) //RED RIGHT
            //far side

    };

    public static final Pose[] PURPLE_PIXEL_POSITIONS = {
            //BLUE FAR LEFT
            //BLUE FAR CENTER
            //BLUE FAR RIGHT
            //RED FAR LEFT
            //RED FAR CENTER
            //RED FAR RIGHT
            //BLUE CLOSE LEFT
            //BLUE CLOSE CENTER
            //BLUE CLOSE RIGHT
            //RED CLOSE LEFT
            //RED CLOSE CENTER
            //RED CLOSE RIGHT
    };

    public static final Pose[] WHITE_PIXEL_POSITIONS = {

    };

}