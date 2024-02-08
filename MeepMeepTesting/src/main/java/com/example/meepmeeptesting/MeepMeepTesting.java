package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    static final HashMap<Integer, Vector2d> aprils = new HashMap<Integer, Vector2d>();

    static {
        aprils.put(1, new Vector2d(-42.83, 63.5));
        aprils.put(2, new Vector2d(-36.83, 63.5));
        aprils.put(3, new Vector2d(-30.83, 63.5));
        aprils.put(4, new Vector2d(30.83, 63.5));
        aprils.put(5, new Vector2d(36.83, 63.5));
        aprils.put(6, new Vector2d(42.83, 63.5));
        aprils.put(7, new Vector2d(43, -72));
        aprils.put(8, new Vector2d(37.5, -72));
        aprils.put(9, new Vector2d(-37.5, -72));
        aprils.put(10,new Vector2d(-43, -72));
    }

    static final HashMap<String, Vector2d> positions = new HashMap<String, Vector2d>();

    static {
        positions.put("p1-1", new Vector2d(30.36, 43.72));
        positions.put("p1-2", new Vector2d(36, 43.72));
        positions.put("p1-3", new Vector2d(42.69, 43.72));
        positions.put("p1-4", new Vector2d(36, 12));
        positions.put("p1-5", new Vector2d(63.62, 44.91));

        positions.put("p2-1", new Vector2d(-41.83, 43.72));
        positions.put("p2-2", new Vector2d(-36, 43.72));
        positions.put("p2-3", new Vector2d(-29.42, 43.72));
        positions.put("p2-4", new Vector2d(-36, 12));
        positions.put("p2-5", new Vector2d(-61.61, 44.91));

        positions.put("p3-1", new Vector2d(-36, -60));
        positions.put("p3-2", new Vector2d(-24, -60));
        positions.put("p3-3", new Vector2d(-12, -60));
        positions.put("p3-4", new Vector2d(-36, -36));
        positions.put("p3-5", new Vector2d(-61.61, -44.16));

        positions.put("p4-1", new Vector2d(12, -60));
        positions.put("p4-2", new Vector2d(24, -60));
        positions.put("p4-3", new Vector2d(36, -60));
        positions.put("p4-4", new Vector2d(36, -36));
        positions.put("p4-5", new Vector2d(63.62, -45.06));

    }

    public static Vector2d orientViaAprilTag(Integer tagID, Integer x, Integer y) {
        return new Vector2d((aprils.get(tagID).getX() - x), (aprils.get(tagID).getY() - y));
    }
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                .splineTo(new Vector2d(-5, 30), Math.toRadians(265))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\evium\\Downloads\\centerstage.png")); }
        catch (IOException ignored) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}