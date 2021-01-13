package org.firstinspires.ftc.teamcode.util.storage

import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.util.InterpLUT
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.pow
import kotlin.math.sqrt

class shooterMode {

    val drive = SampleMecanumDrive(hardwareMap)
    var shooterServo = SimpleServo(hardwareMap, "servo1")

    //InterpLUT for RPM reference (ftclib)
    var RPMlut: InterpLUT = object : InterpLUT() {
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

    fun servoSetGoal() {
        var f = 0.05 // vertical distance between axles
        var a = 1.73 //length of arm
        var c = 2.0 //horizontal axle diff
        var k = 1 // gear ratio

        val targetVectorX = 72.0
        val targetVectorY = 36.0

        val anglelut: InterpLUT = object : InterpLUT() {
            init {

                //Adding each val with a key
                //x is distance, y is angle
                add(5.0, 1.0)
                add(4.1, 0.9)
                add(3.6, 0.75)
                add(2.7, .5)
                add(1.1, 0.2)
                //generating final equation
                createLUT()

            }
        }

        val distance =
            sqrt((targetVectorX - drive.poseEstimate.x).pow(2) + (targetVectorY - drive.poseEstimate.y).pow(2))
        var targetangle = anglelut.get(distance)
        shooterServo.rotateDegrees((90 - kotlin.math.atan2(72.0, -4.25))) //change to equation
    }

}