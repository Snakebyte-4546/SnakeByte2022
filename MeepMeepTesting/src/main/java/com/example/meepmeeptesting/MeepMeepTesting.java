package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(47, 50, 2.1502456665039062, 2.1502456665039062, 15)
                .setDimensions(15,12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))
                                .lineTo(new Vector2d(-35,-35))
                                .splineTo(new Vector2d(-29,-8), Math.toRadians(55))
                                .lineTo(new Vector2d(-31,-10))

                                .lineToLinearHeading(new Pose2d(-38,-11.8, Math.toRadians(180)))
                                .lineTo(new Vector2d(-58,-11.8))
                                .lineToLinearHeading(new Pose2d(-40,-11.8, Math. toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-29,-8, Math.toRadians(55)))
                                .lineToLinearHeading(new Pose2d(-40,-11.8, Math. toRadians(180)))
                                .lineTo(new Vector2d(-58,-11.8))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}