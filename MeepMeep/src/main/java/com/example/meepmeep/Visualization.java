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

        Vector2d parkLeft = new Vector2d(55, -9);
        Vector2d leftAlign = new Vector2d(42, -9);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(178), Math.toRadians(178), 19.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14.00, 61.00, Math.toRadians(-90.00)))
                                .splineTo(new Vector2d(23.21, 55.39), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(41.00, 34.80), Math.toRadians(0.00))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(42.78, 31.96, Math.toRadians(180.00)))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(32.85, 12.60), Math.toRadians(180.00))
                                .splineToConstantHeading(new Vector2d(-37.00, 12.60), Math.toRadians(180.00))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(30.33, 12.90), Math.toRadians(180.00))
                                .lineToLinearHeading(new Pose2d(39.08, 28.84, Math.toRadians(0.00)))
                                .build()


                        /*
                        .lineToLinearHeading(new Pose2d(47.6, 54.3, Math.toRadians(-180)))
                                .lineToLinearHeading(new Pose2d(58.7, 58.7, Math.toRadians(-180)))
                         */
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}