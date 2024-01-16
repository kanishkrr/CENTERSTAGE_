package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Visualization {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d yellow = new Pose2d(46.8, 37, Math.toRadians(0));

        Vector2d parkLeft = new Vector2d(55, 58.6);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(150), Math.toRadians(150), 17.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 61, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(-42, 59))
                        .lineToLinearHeading(new Pose2d(-35.3, 34.3, Math.toRadians(0)))
                        .waitSeconds(7)
                        .back(8)
                        .splineToConstantHeading(new Vector2d(-21.9, 8.5), Math.toRadians(0))
                        .forward(26)
                        .splineToConstantHeading(new Vector2d(44.9, 36.2), Math.toRadians(0))
                        .strafeLeft(18)
                        .turn(Math.toRadians(180))
                        .strafeTo(parkLeft)
                        .build()


                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}