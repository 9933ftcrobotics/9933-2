package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15, 62, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-5, 36))
                .strafeTo(new Vector2d(-5, 45))
                .splineToLinearHeading(new Pose2d(-36, 25, Math.toRadians(180)), 5)
                .lineToYSplineHeading(55, Math.toRadians(90))
                .turnTo(Math.toRadians(180))
                                .strafeTo(new Vector2d(-45, 25))
                        .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-58, 50))
                .strafeTo(new Vector2d(-45, 45))
                .strafeTo(new Vector2d(-45, 55))
                .strafeTo(new Vector2d(-5, 45))
                                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(-5, 36))
                .strafeTo(new Vector2d(-5, 45))
                .strafeTo(new Vector2d(-47, 55))
                /*.strafeTo(new Vector2d(-36, 45))
                .strafeTo(new Vector2d(-46, 46))
                        .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-50, 50))
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(-54, 46))
                .strafeTo(new Vector2d(-58, 46))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-60, 50))
                .strafeTo(new Vector2d(-50, 45))
                .strafeTo(new Vector2d(-50, 50))
                .strafeTo(new Vector2d(-5, 45))
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(-5, 36))
                .strafeTo(new Vector2d(-5, 45))
                .strafeTo(new Vector2d(-53, 58))*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}