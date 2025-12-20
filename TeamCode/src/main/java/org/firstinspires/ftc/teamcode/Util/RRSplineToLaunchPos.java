package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.FIELD_HALF;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.mtoin;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RRSplineToLaunchPos {

    public static void splineLaunchPos(MecanumDrive drive, Pose2d pose, double angledeg) {
        Action move = drive.actionBuilder(pose)
                .splineTo(new Vector2d(-22.5, 22), Math.toRadians(angledeg))
                .build();
        Actions.runBlocking(move);

         /*
        Action move = drive.actionBuilder(pose)
                .strafeTo(new Vector2d(-22.5,22))
                .build();
        Actions.runBlocking(move);*/
    }
}
