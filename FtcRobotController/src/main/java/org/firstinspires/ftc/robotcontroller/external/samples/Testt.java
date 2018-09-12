package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name = "Testt", group = "Pushbot")
public class Testt extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();

    RangeSensorWrapper rangeSensorA;
    RangeSensorWrapper rangeSensorB;

    I2cAddr a1 = I2cAddr.create8bit(0x28);
    I2cAddr a2 = I2cAddr.create8bit(0x26);


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("a1", a1.get8Bit());
        telemetry.addData("a2", a2.get8Bit());

        //  rangeSensorA = new RangeSensorWrapper("sensor_rangeA", hardwareMap, a1, telemetry);
        telemetry.addData("rangesensorA initializat", "OK");
        telemetry.update();
        //sleep(2000);
        //  rangeSensorB = new RangeSensorWrapper("sensor_rangeB", hardwareMap, a2, telemetry);
        telemetry.addData("rangesensorB initializat", "OK");
        telemetry.update();
        //sleep(2000);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("A", rangeSensorA.getDistanceInCm());
            telemetry.update();

            telemetry.addData("B", rangeSensorB.getDistanceInCm());
            telemetry.update();

        }


    }


}
