package org.firstinspires.ftc.teamcode.util.EOCVtests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera

/*
    Beta 10669 T265 localizer (not for use yet)
    This uses the ftc265 wrapper to create a localizer that merges the measures from the t265 and deadwheel odometry
 */

class T265localizerV2(hardwareMap: HardwareMap) : Localizer {

    //convert FTClib pose to RR pose,  convert RR pose to FTClib pose \
    fun com.arcrobotics.ftclib.geometry.Pose2d.toRR(): Pose2d =
        Pose2d(this.translation.y, this.translation.x, this.heading)

    fun Pose2d.toFTCLib(): com.arcrobotics.ftclib.geometry.Pose2d =
        com.arcrobotics.ftclib.geometry.Pose2d(-this.x, -this.y, Rotation2d(this.heading))

    //function to make something to radians
    fun Double.toRadians(): Double = Math.toRadians(this)

    //Get x, get y, get rotation in degrees and radians
    val com.arcrobotics.ftclib.geometry.Pose2d.x: Double get() = this.translation.x
    val com.arcrobotics.ftclib.geometry.Pose2d.y: Double get() = this.translation.y
    val com.arcrobotics.ftclib.geometry.Pose2d.rotationDeg: Double get() = this.rotation.degrees
    val com.arcrobotics.ftclib.geometry.Pose2d.rotationRad: Double get() = this.rotation.radians

    //get ftclib chassis speeds and put it in rr
    fun ChassisSpeeds.toRRPose2d(): Pose2d =
        Pose2d(this.vxMetersPerSecond, this.vyMetersPerSecond, this.omegaRadiansPerSecond)

    //convert to inches (for rr - ftclib)
    fun Pose2d.toInches(): Pose2d = Pose2d(this.x / 0.025400051, this.y / 0.025400051, this.heading)

    //convert to meters (rr to ftclib)
    fun com.arcrobotics.ftclib.geometry.Pose2d.toMeters(): com.arcrobotics.ftclib.geometry.Pose2d =
        com.arcrobotics.ftclib.geometry.Pose2d(this.x * 0.025400051, this.y * 0.025400051, Rotation2d(this.heading))


    //distance between camera and robot (pose)
    private val cameraToRobot = Transform2d(Translation2d(3.0, 0.0), Rotation2d(0.0))
    val startingPose: com.arcrobotics.ftclib.geometry.Pose2d =
        com.arcrobotics.ftclib.geometry.Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))

    //map camera, add odo covariance (50% t265 50% deadwheel)
    var T265Camera = T265Camera(cameraToRobot, 0.5, hardwareMap.appContext)

    //robot pose estimate
    override var poseEstimate: Pose2d
        get() = Pose2d(
            T265Camera.lastReceivedCameraUpdate.pose.toRR().toInches().x,
            T265Camera.lastReceivedCameraUpdate.pose.toRR().toInches().y,
            -T265Camera.lastReceivedCameraUpdate.pose.toRR().toInches().heading
        )
        set(value) {
            T265Camera.setPose(value.toFTCLib().toMeters())
        }

    //current velocity (kinda unnecessary)
    override val poseVelocity: Pose2d
        get() = T265Camera.lastReceivedCameraUpdate.velocity.toRRPose2d().toInches()

    //completes an update
    override fun update() {
        T265Camera.lastReceivedCameraUpdate
    }

    //stops the camera
    fun stopCamera() {
        T265Camera.stop()
    }

    fun startCamera() {


    }


}
