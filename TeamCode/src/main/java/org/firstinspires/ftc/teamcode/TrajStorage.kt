package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.util.InterpLUT
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.atan

class TrajStorage {
    //var targetPosition = Vector2d(72.0, 36.0)
    var angleTheta = 90 - atan((105 / 24).toDouble())
    var myPose = Pose2d(-62.0, -50.0, Math.toRadians(0.0))
    var drive = SampleMecanumDrive(BlocksOpModeCompanion.hardwareMap)

    var lut: InterpLUT = object : InterpLUT() {
        init {

            //Adding each val with a key
            add(5.0, 1.0)
            add(4.1, 0.9)
            add(3.6, 0.75)
            add(2.7, .5)
            add(1.1, 0.2)
            //generating final equation
            createLUT()
        }
    }

    //go to mat A
    var trajectoryA1Red1 = drive.trajectoryBuilder(
            Pose2d(-62.0, -50.0, Math.toRadians(0.0)))
            .splineTo(Vector2d(0.0, -60.0), Math.toRadians(0.0)) //.splineTo(new Vector2d(52.0, -60.0), 0.0)
            .build()

    //pick up rings on the way to the second wobble goal
    var trajectoryA1Red2 = drive.trajectoryBuilder(
            Pose2d(0.0, -60.0, Math.toRadians(0.0)))
            .lineToSplineHeading(Pose2d(-20.0, -36.0, -angleTheta))
            .build()

    //continue to the second wobble goal
    var trajectoryA1Red3 = drive.trajectoryBuilder(
            Pose2d(-20.0, -36.0, Math.toRadians(-angleTheta)))
            .lineToSplineHeading(Pose2d(-60.0, -25.0, Math.toRadians(0.0)))
            .build()

    //pick up wobble and aim to powershots
    var trajectoryA1Red4 = drive.trajectoryBuilder(
            Pose2d(-60.0, -25.0, Math.toRadians(0.0)))
            .splineTo(Vector2d(-23.0, -36.0), angleTheta)
            .build()

    //Back to mat A to drop off second one
    var trajectoryA1Red5 = drive.trajectoryBuilder(
            Pose2d(-23.0, -36.0, Math.toRadians(angleTheta))) //.splineTo(new Vector2d(0,-60), Math.toRadians(0))
            .lineToSplineHeading(Pose2d(0.0, -60.0, Math.toRadians(0.0)))
            .build()

    //Park on line
    var trajectoryA1Red6 = drive.trajectoryBuilder(
            Pose2d(0.0, -60.0, Math.toRadians(0.0)))
            .splineToConstantHeading(Vector2d(10.0, -60.0), 0.0)
            .build()

    //go to mat B
    var trajectoryB1Red1 = drive.trajectoryBuilder(
            Pose2d(-62.0, -50.0, Math.toRadians(0.0)))
            .splineTo(Vector2d(34.0, -35.0), 0.0)
            .build()

    //Back to mat B to drop off second one
    var trajectoryB1Red5 = drive.trajectoryBuilder(
            Pose2d(-23.0, -36.0, Math.toRadians(angleTheta)))
            .lineToSplineHeading(Pose2d(34.0, -35.0, Math.toRadians(0.0)))
            .build()

    //go to mat C
    var trajectoryC1Red1 = drive.trajectoryBuilder(
            Pose2d(-62.0, -50.0, Math.toRadians(0.0)))
            .splineTo(Vector2d(52.0, -60.0), 0.0)
            .build()

    //Back to mat C to drop off second one
    var trajectoryC1Red5 = drive.trajectoryBuilder(
            Pose2d(-23.0, -36.0, Math.toRadians(angleTheta)))
            .lineToSplineHeading(Pose2d(54.0, -60.0, Math.toRadians(0.0)))
            .build()
}