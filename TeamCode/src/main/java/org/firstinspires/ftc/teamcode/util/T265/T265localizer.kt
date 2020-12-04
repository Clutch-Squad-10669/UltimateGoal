package org.firstinspires.ftc.teamcode.util.T265

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera

class T265localizer(hardwareMap: HardwareMap) : Localizer {
    val cameraToRobot = Transform2d(Translation2d(12.0, 0.0), Rotation2d(180.0))
    val startingPose: com.arcrobotics.ftclib.geometry.Pose2d = com.arcrobotics.ftclib.geometry.Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
    var T265Camera = T265Camera(cameraToRobot, 0.5, hardwareMap.appContext)
    /**
     * Current robot pose estimate.
     */
    override var poseEstimate: Pose2d
        get() = Pose2d(T265Camera.lastReceivedCameraUpdate.pose.toRR().toInches().x, T265Camera.lastReceivedCameraUpdate.pose.toRR().toInches().y, -T265Camera.lastReceivedCameraUpdate.pose.toRR().toInches().heading)
        set(value) {
            T265Camera.setPose(value.toFTCLib().toMeters())
        }

    /**
     * Current robot pose velocity (optional)
     */
    override val poseVelocity: Pose2d
        get() = T265Camera.lastReceivedCameraUpdate.velocity.toRRPose2d().toInches()

    /**
     * Completes a single localization update.
     */
    override fun update() {
        T265Camera.lastReceivedCameraUpdate
    }

    fun stopCamera() {
        T265Camera.stop()
    }

    fun startCamera(){


    }

}


fun com.arcrobotics.ftclib.geometry.Pose2d.toRR(): Pose2d = Pose2d(this.translation.y, this.translation.x, this.heading)

fun Pose2d.toFTCLib(): com.arcrobotics.ftclib.geometry.Pose2d = com.arcrobotics.ftclib.geometry.Pose2d(-this.x, -this.y, Rotation2d(this.heading))

fun Double.toRadians(): Double = Math.toRadians(this)

val com.arcrobotics.ftclib.geometry.Pose2d.x: Double
    get() = this.translation.x

val com.arcrobotics.ftclib.geometry.Pose2d.y: Double
    get() = this.translation.y

val com.arcrobotics.ftclib.geometry.Pose2d.rotationDeg: Double
    get() = this.rotation.degrees

val com.arcrobotics.ftclib.geometry.Pose2d.rotationRad: Double
    get() = this.rotation.radians

fun ChassisSpeeds.toRRPose2d(): Pose2d = Pose2d(this.vxMetersPerSecond, this.vyMetersPerSecond, this.omegaRadiansPerSecond)

fun Pose2d.toInches(): Pose2d = Pose2d(this.x / 0.025400051, this.y / 0.025400051, this.heading)

fun com.arcrobotics.ftclib.geometry.Pose2d.toMeters(): com.arcrobotics.ftclib.geometry.Pose2d = com.arcrobotics.ftclib.geometry.Pose2d(this.x * 0.025400051, this.y * 0.025400051, Rotation2d(this.heading))