package org.firstinspires.ftc.teamcode.util.storage

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.lang.Math.atan2

class TrajStorage {

    //angles for powershots
    var angleTheta = (90 - atan2(72.0, -4.25))
    var angleTheta1 = (90 - atan2(72.0, -11.75))
    var angleTheta2 = (90 - atan2(72.0, -19.25))

    //start pose for most of them
    var myPose = Pose2d(-62.0, -50.0, Math.toRadians(0.0))

    //just to get rid of errors 
    val drive = SampleMecanumDrive(hardwareMap)

    //general shared trajectories for the 2nd and 3rd squares
    //go to mat A
    var trajectoryA1Red1 = drive.trajectoryBuilder(
        Pose2d(-62.0, -50.0, Math.toRadians(0.0))
    )
        .splineTo(Vector2d(0.0, -60.0), Math.toRadians(0.0)) //.splineTo(new Vector2d(52.0, -60.0), 0.0)
        .build()

    //pick up rings on the way to the second wobble goal
    var trajectoryA1Red2 = drive.trajectoryBuilder(
        Pose2d(0.0, -60.0, Math.toRadians(0.0))
    )
        .lineToSplineHeading(Pose2d(-20.0, -36.0, -angleTheta))
        .build()

    //continue to the second wobble goal
    var trajectoryA1Red3 = drive.trajectoryBuilder(
        Pose2d(-20.0, -36.0, Math.toRadians(-angleTheta))
    )
        .lineToSplineHeading(Pose2d(-60.0, -25.0, Math.toRadians(0.0)))
        .build()

    //pick up wobble and aim to powershots
    var trajectoryA1Red4 = drive.trajectoryBuilder(
        Pose2d(-60.0, -25.0, Math.toRadians(0.0))
    )
        .splineTo(Vector2d(-23.0, -36.0), angleTheta)
        .build()

    //Back to mat A to drop off second one
    var trajectoryA1Red5 = drive.trajectoryBuilder(
        Pose2d(-23.0, -36.0, Math.toRadians(angleTheta))
    ) //.splineTo(new Vector2d(0,-60), Math.toRadians(0))
        .lineToSplineHeading(Pose2d(0.0, -60.0, Math.toRadians(0.0)))
        .build()

    //Park on line
    var trajectoryA1Red6 = drive.trajectoryBuilder(
        Pose2d(0.0, -60.0, Math.toRadians(0.0))
    )
        .splineToConstantHeading(Vector2d(10.0, -60.0), 0.0)
        .build()

    //stuff for the Second Mat
    //go to mat B
    var trajectoryB1Red1 = drive.trajectoryBuilder(
        Pose2d(-62.0, -50.0, Math.toRadians(0.0))
    )
        .splineTo(Vector2d(34.0, -35.0), 0.0)
        .build()

    //Back to mat B to drop off second one
    var trajectoryB1Red5 = drive.trajectoryBuilder(
        Pose2d(-23.0, -36.0, Math.toRadians(angleTheta))
    )
        .lineToSplineHeading(Pose2d(34.0, -35.0, Math.toRadians(0.0)))
        .build()

    //stuff for the 3rd mat
    //go to mat C
    var trajectoryC1Red1 = drive.trajectoryBuilder(
        Pose2d(-62.0, -50.0, Math.toRadians(0.0))
    )
        .splineTo(Vector2d(52.0, -60.0), 0.0)
        .build()

    //Back to mat C to drop off second one
    var trajectoryC1Red5 = drive.trajectoryBuilder(
        Pose2d(-23.0, -36.0, Math.toRadians(angleTheta))
    )
        .lineToSplineHeading(Pose2d(54.0, -60.0, Math.toRadians(0.0)))
        .build()

    //trajectory for first mat (since no rings)
    var a1red1 = drive.trajectoryBuilder(
        Pose2d(-62.0, -50.0, Math.toRadians(0.0))
    )
        .splineTo(Vector2d(-5.0, -57.0), 0.0)
        .build()


    var a1red2 = drive.trajectoryBuilder(
        Pose2d(-5.0, -57.0, Math.toRadians(0.0))
    )
        .lineToSplineHeading(Pose2d(-62.0, -50.0, 0.0))
        .build()


    var a1red3 = drive.trajectoryBuilder(
        Pose2d(-62.0, -50.0, Math.toRadians(0.0))
    )
        .lineToSplineHeading(Pose2d(-62.0, -25.0, 0.0))
        .build()


    var a1red4 = drive.trajectoryBuilder(
        Pose2d(-62.0, -25.0, Math.toRadians(0.0))
    )
        .splineTo(Vector2d(-23.0, -36.0), angleTheta)
        .build()

    var a1red5 = drive.trajectoryBuilder(
        Pose2d(-23.0, -36.0, Math.toRadians(angleTheta))
    )
        .lineToSplineHeading(Pose2d(-23.1, -36.1, Math.toRadians(angleTheta1)))
        .build()

    var a1red6 = drive.trajectoryBuilder(
        Pose2d(-23.1, -36.1, Math.toRadians(angleTheta1))
    )
        .lineToSplineHeading(Pose2d(-23.2, -36.2, Math.toRadians(angleTheta2)))
        .build()


    var a1red7 = drive.trajectoryBuilder(
        Pose2d(-23.2, -36.2, Math.toRadians(angleTheta2))
    )
        .lineToSplineHeading(Pose2d(10.0, -40.0, Math.toRadians(-90.0)))
        .build()


}
