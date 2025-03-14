package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.25, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-38, -61, Math.toRadians(0)))
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)), Math.toRadians(180)) //drop off pre loaded
//                        .setTangent(Math.toRadians(45))
//                        .splineTo(new Vector2d(-48,-38.5),Math.toRadians(90))//to first sample; rotation 0.5
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-55,-53),Math.toRadians(229.5)) //drop off sample 1
//                        .setReversed(false)
//                        .setTangent(Math.toRadians(100))
//                        .splineToLinearHeading(new Pose2d(-58,-39.3,Math.toRadians(90)),Math.toRadians(107)) //to second sample; rotation 0.75
//                        .setReversed(true)
//                        .setTangent(Math.toRadians(300))
//                        .splineToLinearHeading(new Pose2d(-55,-53.5,Math.toRadians(48)),Math.toRadians(232)) //drop off sample 2
//                        .setReversed(false)
//                        .setTangent(Math.toRadians(80))
//                        .splineToLinearHeading(new Pose2d(-55,-40, Math.toRadians(130)), Math.toRadians(90))
//                        .setReversed(true)
//                        .setTangent(Math.toRadians(280))
//                        .splineToLinearHeading(new Pose2d(-55,-53,Math.toRadians(48)), Math.toRadians(230))// to third sample; rotation 0.95/1

//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52,-53),Math.toRadians(225)) //drop off sample 3
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-30,-11),Math.toRadians(0))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}