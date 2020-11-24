package org.firstinspires.ftc.teamcode.trajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import java.lang.Math.atan

object TrajectoryGen1 {

    var angleTheta = 90 - atan((105 / 24).toDouble())

    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()
        val trajectoryA1Red1 = TrajectoryBuilder(Pose2d(-62.0, -50.0, 0.0), 0.0, combinedConstraints)
        val trajectoryA1Red2 = TrajectoryBuilder(Pose2d(0.0, -60.0, 0.0), 0.0, combinedConstraints)
        val trajectoryA1Red3 = TrajectoryBuilder(Pose2d(-20.0, -36.0, -angleTheta), 0.0, combinedConstraints)
        val trajectoryA1Red4 = TrajectoryBuilder(Pose2d(-60.0, -25.0, 0.0), 0.0, combinedConstraints)
        val trajectoryA1Red5 = TrajectoryBuilder(Pose2d(-23.0, -36.0, angleTheta), 0.0, combinedConstraints)
        val trajectoryA1Red6 = TrajectoryBuilder(Pose2d(-0.0, -60.0, 0.0), 0.0, combinedConstraints)



        trajectoryA1Red1
                .splineTo(Vector2d(0.0, -60.0), 0.0)
        trajectoryA1Red2
                .lineToSplineHeading(Pose2d(-20.0, -36.0, -angleTheta))
        trajectoryA1Red3
                .lineToSplineHeading(Pose2d(-60.0, -25.0, 0.0))
        trajectoryA1Red4
                .splineTo(Vector2d(-23.0, -36.0), angleTheta)
        trajectoryA1Red5
                .lineToSplineHeading(Pose2d(0.0, -60.0, 0.0))
        trajectoryA1Red6
                .splineToConstantHeading(Vector2d(10.0, -60.0), 0.0)



        list.add(trajectoryA1Red1.build())
        list.add(trajectoryA1Red2.build())
        list.add(trajectoryA1Red3.build())
        list.add(trajectoryA1Red4.build())
        list.add(trajectoryA1Red5.build())
        list.add(trajectoryA1Red6.build())

        return list
    }


}

val Double.toRadians get() = (Math.toRadians(this))