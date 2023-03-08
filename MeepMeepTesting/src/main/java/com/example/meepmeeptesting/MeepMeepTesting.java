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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15,12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))
                                .lineTo(new Vector2d(-35,-35))
                                .splineTo(new Vector2d(-28,-8), Math.toRadians(65))
                                .addDisplacementMarker(() -> {
                                    //robot.moveLift(0.5, "high");
                                    //robot.wait(1);
                                    //robot.clamp(false);
                                })

                                .lineToLinearHeading(new Pose2d(-38,-12.25, Math.toRadians(180)))
                                .lineTo(new Vector2d(-58,-12.25))
                                .addDisplacementMarker(() -> {
                                    //robot.moveLift(0.5, "s5");
                                    //robot.wait(1);
                                    //robot.clamp(true);
                                })
                                .lineToLinearHeading(new Pose2d(-40,-12.25, Math. toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-28,-8, Math.toRadians(65)))
                                .addDisplacementMarker(() -> {
                                    //robot.wait(1);
                                    //robot.clamp(false);
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}