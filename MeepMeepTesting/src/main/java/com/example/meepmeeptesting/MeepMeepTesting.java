package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(72, 50, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61.3, 24, Math.PI))
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(180))
                .strafeTo(new Vector2d(-31,31)).turnTo(Math.toRadians(135))
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-11.4,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-11.4,58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-11.4,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-31,31), Math.toRadians(135))
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(12,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(12,58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(12,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-31,31), Math.toRadians(135))
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-9,23), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}