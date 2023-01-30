package com.example.meepmeeptesting;

import java.util.Collections;
import java.util.Vector;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.TurnSegment;

public class MeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);


        double  east = Math.toRadians(0);
        double  northEast = Math.toRadians(45);
        double  north = Math.toRadians(90);
        double  northWest = Math.toRadians(125);
        double  west = Math.toRadians(180);
        double  southWest = Math.toRadians(205);
        double  south = Math.toRadians(270);
        double  southEast = Math.toRadians(295);

        Pose2d startPose = new Pose2d(-35, -64, north);
        Pose2d scorePose= new Pose2d(-29, -5, northWest);
        Pose2d pickupPose = new Pose2d(-60, -12, east);
        Vector2d nutralVector = new Vector2d(-36 -12);


        RoadRunnerBotEntity positionTester = new DefaultBotBuilder(meepMeep)
                .setConstraints(35, 20, Math.toRadians(100), Math.toRadians(250), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(startPose)

                                    //first
                                    .lineTo(nutralVector)
                                    .lineToSplineHeading(poseMaker(nutralVector, Math.toRadians(359)))

                                    .forward(.1)

                                    .build();
                        }
                );


        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel     maxAccel     maxAngVel            maxAngAccel          track width
                .setConstraints(35, 20, Math.toRadians(100), Math.toRadians(250), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(startPose)


                                    // preloadToNutral
                                    .strafeTo(nutralVector)
                                    .lineToSplineHeading(poseMaker(nutralVector, northWest))

                                    // score1
                                    .lineToSplineHeading(scorePose)
                                    .strafeTo(new Vector2d(-58, -12))

                                    //.strafeTo(new Vector2d(-36, -12))
                                    //.turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                                    .build();
                        }
                );


        RoadRunnerBotEntity drift = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel     maxAccel     maxAngVel            maxAngAccel          track width
                .setConstraints(45, 45, Math.toRadians(305), Math.toRadians(300), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))

                                    //drop off first cone

                                    .forward(5)
                                    .splineToSplineHeading(new Pose2d(-29, -15, Math.toRadians(220)), Math.toRadians(62))

                                    .build();
                        }
                );


        RoadRunnerBotEntity motionCalibration = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))

                                    .strafeTo(new Vector2d(-35, -12.5))
                                    //     angle to turn to  |   turn velocity   |  turn acceleration
                                    .turn(0            , Math.toRadians(80), Math.toRadians(150))
                                    .turn(Math.toRadians(90))
                                    .turn(Math.toRadians(270))
                                    .strafeTo(new Vector2d(-12, -12.5))
                                    .lineToSplineHeading(new Pose2d(-12, -40, Math.toRadians(270)))
                                    .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(180)), Math.toRadians(180))
                                    .splineToSplineHeading(new Pose2d(-35, -50, Math.toRadians(90)), Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(-48, -34), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-60, -50, Math.toRadians(270)), Math.toRadians(270))
                                    .strafeTo(new Vector2d(-60, -54))

                                    .lineToLinearHeading(new Pose2d(-35, -64, Math.toRadians(90)))
                                    .build();
                        }
                );



        /*Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        Pose2d scorePose= new Pose2d(-29, -5, Math.toRadians(220));
        Pose2d pickupPose = new Pose2d(-60, -12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


        // Trajectory setup
        RoadRunnerBotEntity preloadToGoal = drive.trajectorySequenceBuilder(startPose)
            return drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    //prime();
                })
                .strafeTo(new Vector2d(-35, -10.2))
                .turn(Math.toRadians(130), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-28.5, -4.5))
                .build();



        RoadRunnerBotEntity park1 = drive.trajectorySequenceBuilder(preloadToGoal.end())
            return drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-36, -12)) // neutral pos
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150)) // neutral heading
                .strafeTo(new Vector2d(-62, -12)) //park pos
                .build();

        RoadRunnerBotEntity park2 = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-36, -12)) // neutral pos
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150)) // neutral heading
                .build();

        RoadRunnerBotEntity park3 = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-36, -12)) //neutral
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-12, -12))
                .build();*/




        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(positionTester) //  test | drift | motionCalibration | positionTester
                .start();
    }
        private static Pose2d poseMaker(Vector2d cord, double head) {
            return new Pose2d(cord.getX() + .01, cord.getY(), head);
        }


}
