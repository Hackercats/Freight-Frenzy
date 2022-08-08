package com.example.meepmeeptestingtwo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingTwo {

    static double originToWall = 141.0/2.0; // I guess the field is actually 141 inches wide
    static double wallDistance = originToWall - 6.5; // Center of bot is 7in from wall

    static Pose2d startPos = new Pose2d(40,-(wallDistance), Math.toRadians(0));

    static Pose2d preloadDepositPos = new Pose2d(0,-38,Math.toRadians(-45));

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 700 pixels at 50 fps
        MeepMeep meepMeep = new MeepMeep(740,60);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep) // Create a bot
                .setDimensions(11.6,18) // Width of 11.6 to match our thin bot
                .setConstraints(55, 30, Math.toRadians(167), Math.toRadians(167), 11)
                // The path we are simulating
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineToSplineHeading(new Pose2d(20,startPos.getY(),0))
                                .splineToSplineHeading(preloadDepositPos,Math.toRadians(120))
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(20,startPos.getY(),0),0)
                                .lineToSplineHeading(startPos)
                                .build()
                );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(0.9f)
                .addEntity(bot)
                .start();
    }
}