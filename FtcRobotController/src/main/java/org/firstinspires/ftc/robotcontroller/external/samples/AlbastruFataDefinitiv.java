package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Camera;
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
import com.vuforia.CameraDevice;
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



@Autonomous(name="AFD", group="Pushbot")
public class AlbastruFataDefinitiv extends LinearOpMode {


    HardwarePushbot robot = new HardwarePushbot();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime io      = new ElapsedTime();


    ColorSensorWrapper color_sensor1;
    ColorSensorWrapper color_sensor2;
    ColorSensorWrapper color_sensor3;
    RangeSensorWrapper range_sensors;
    RangeSensorWrapper range_sensorf;
    ModernRoboticsI2cGyro gyro;
    I2cAddr a1 = I2cAddr.create8bit(0x28);
    I2cAddr a2 = I2cAddr.create8bit(0x26);
    public char Vuforia_cit;
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;
    static final double TOLERANCE = 3;
    OpenGLMatrix lastLocation = null; // WARNING: VERY  INACCURATE, USE ONLY TO ADJUST TO FIND IMAGE AGAIN! DO NOT BASE MAJOR MOVEMENTS OFF OF THIS!!
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        color_sensor1 = new ColorSensorWrapper("color_sensor1", hardwareMap, 0x3c, telemetry);
        color_sensor1.enableLed();

        color_sensor2 = new ColorSensorWrapper("color_sensor2", hardwareMap, 0x3e, telemetry);
        color_sensor2.enableLed();

        color_sensor3 = new ColorSensorWrapper("color_sensor3", hardwareMap, 0x3a, telemetry);
        color_sensor3.enableLed();
      //  range_sensors = new RangeSensorWrapper("spate", hardwareMap, a1, telemetry);
      //  range_sensorf = new RangeSensorWrapper("fata", hardwareMap, a2, telemetry);

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");


        timer.reset();
        while (!isStopRequested() && gyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated.");
        telemetry.clear();
        telemetry.update();


        waitForStart();


        // if (!opModeIsActive()) return;
        // JewelsGo(1, 3000, 1000, 1000, 3000);
        // sleep(500);

        Vuforia_cit = citireVuforia();

        if (Vuforia_cit == 'c') {

            if (!opModeIsActive()) return;
            DriveBack(0.01, 10);
            if (!opModeIsActive()) return;
            DriveBack(0.3, 900);
            sleep(500);
            if (!opModeIsActive()) return;
            Rotire90(0.3, 225);
            sleep(500);
            if (!opModeIsActive()) return;
            DriveBack(0.1, 10);
            if (!opModeIsActive()) return;
            DriveBack(0.3, 200);
            if (!opModeIsActive()) return;
            basculare(0.90);
            sleep(50);
            DriveBack(0.1, 50);
            sleep(50);
            GoTo(0.1, 50);
            if (!opModeIsActive()) return;
            basculare(0.25);

        }
        if (Vuforia_cit == 'r') {
            if (!opModeIsActive()) return;
            DriveBack(0.01, 10);
            if (!opModeIsActive()) return;
            DriveBack(0.3, 1000);
            if (!opModeIsActive()) return;
            Rotire270(0.3, 770);
            if (!opModeIsActive()) return;
            GoTo(0.1, 10);
            if (!opModeIsActive()) return;
            GoTo(0.2, 1500);
            if (!opModeIsActive()) return;
            Rotire90(0.3, 1200);
            if (!opModeIsActive()) return;
            DriveBack(0.1, 200);
            sleep(500);
            if (!opModeIsActive()) return;
            basculare(0.90);
            if (!opModeIsActive()) return;
            DriveBack(0.1, 100);
            Go();
            if (!opModeIsActive()) return;
            basculare(0.25);
        }
        if (Vuforia_cit == 'l') {
            DriveBack(0.01, 10);
            if (!opModeIsActive()) return;
            DriveBack(0.3, 950);
            if (!opModeIsActive()) return;
            Rotire90(0.3, 750);
            sleep(500);
            if (!opModeIsActive()) return;
            GoTo(0.2, 300);
            sleep(500);
            if (!opModeIsActive()) return;
            Rotire270(0.3, 700);
            if (!opModeIsActive()) return;
            DriveBack(0.2, 125);
            sleep(50);
            basculare(0.90);
            sleep(50);
            DriveBack(0.2, 80);
            sleep(50);
            if (!opModeIsActive()) return;
            sleep(50);
            Go();
            if (!opModeIsActive()) return;
            basculare(0.25);
        }

    }

    public void scuipa(){
        if (opModeIsActive()){
            io.reset();
            while(io.seconds() < 2){
                robot.cubst.setPower(-1);
                robot.cubdr.setPower(-1);
            }
            robot.roatast.setPower(0);
            robot.roatadr.setPower(0);
        }

    }


    public void Go(){
        if (opModeIsActive()){
            io.reset();
            while(io.seconds() < 2){
                robot.roatast.setPower(-0.05);      //USE IT ONLY IN SMALL-SPACED LOCATIONS
                robot.roatadr.setPower(-0.05);
            }
            robot.roatast.setPower(0);
            robot.roatadr.setPower(0);}}

    public void GoTo(double speed, int counts) {



        robot.roatadr.setDirection(DcMotor.Direction.REVERSE);
        robot.roatast.setDirection(DcMotor.Direction.FORWARD);

        if (!opModeIsActive()) return;         ////used for quiting the stuck in stop() error

        if (counts < 5) {
            GoTo(0.1, 10);      ////give the wheels an impulse in order to move at the same time next command
        }                                     ////use as a different command if the desired position is bigger than 5 counts
        int newLeftTarget = 0;
        int newRightTarget = 0;

        if (opModeIsActive()) {

            newLeftTarget = robot.roatast.getCurrentPosition() + counts;        //COUNTS NEEDED IN ORDER TO GET TO THE DESIRED POSITION
            newRightTarget = robot.roatadr.getCurrentPosition() + counts;
            robot.roatast.setTargetPosition(newLeftTarget);
            robot.roatadr.setTargetPosition(newRightTarget);

            robot.roatast.setPower(speed);
            robot.roatadr.setPower(speed);

            while (opModeIsActive() &&
                    (robot.roatast.isBusy() || robot.roatadr.isBusy())
                    && (robot.roatast.getCurrentPosition() <= newLeftTarget         //KEEP LOOPING UNTIL THE ROBOT REACHES THE DESIRED POSITION
                    || robot.roatadr.getCurrentPosition() <= newRightTarget)) {

                if(counts == 1600){
                    if (range_sensors.getDistanceInCm() > 135) {
                        robot.roatast.setPower(0);
                        robot.roatadr.setPower(0);
                    }
                } else if (counts == 1300){
                    if (range_sensors.getDistanceInCm() > 120) {            //CHECK THE DESIRED POSITION WITH THE RANGE SENSORS
                        robot.roatast.setPower(0);
                        robot.roatadr.setPower(0);
                    }
                } else if (counts == 1000) {
                    if (range_sensors.getDistanceInCm() > 115) {
                        robot.roatast.setPower(0);
                        robot.roatadr.setPower(0);
                    }
                }
            }

            robot.roatast.setPower(0);
            robot.roatadr.setPower(0);

            robot.roatast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.roatast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void DriveBack(double speed, int counts) {
        if (!opModeIsActive()) return;

        int newLeftTarget = 0;
        int newRightTarget = 0;


        robot.roatadr.setDirection(DcMotor.Direction.FORWARD);
        robot.roatast.setDirection(DcMotor.Direction.REVERSE);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.roatast.getCurrentPosition() + counts;
            newRightTarget = robot.roatadr.getCurrentPosition() + counts;
            robot.roatast.setTargetPosition(newLeftTarget);
            robot.roatadr.setTargetPosition(newRightTarget);



            robot.roatast.setPower(speed);
            //sleep(1);
            robot.roatadr.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.roatast.isBusy() || robot.roatadr.isBusy())
                    && (robot.roatast.getCurrentPosition() <= newLeftTarget
                    || robot.roatadr.getCurrentPosition() <= newRightTarget)) {


            }

            // Stop all motion;
            robot.roatast.setPower(0);
            robot.roatadr.setPower(0);

            robot.roatast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.roatast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public char citireVuforia() {

        OpenGLMatrix lastLocation = null; // WARNING: VERY INACCURATE, USE ONLY TO ADJUST TO FIND IMAGE AGAIN! DO NOT BASE MAJOR MOVEMENTS OFF OF THIS!!

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVURij3/////AAAAGWusRd1rFkGagFOB+18PAqEfWIuxX6zFovEtmuPxW4RAP96Bmsn5uiTeFl/FRiP8Zcwp9podgVtd7GLTaMzz3zNIMf02Pjk9SwnovU5aYFXEdLK6fuVnqGHoSwn09fYF0aZ+zJ4wpidGnE6zcm+D4A6nbqjKqIdvy7kEBVC6nv/iN61N6q+Qt2Vw99cAwDcjSs7JgPwZBjl5oBMO89JBmpf+OgMzwFd/KFo1GqD4eL8PXEZd8Yvs6aB9IMeBuNTRkruXc7fVcUPqab8ysWZjlhO2F6jZ9pV84uQmoTFlMf3wY8U8wg4jfIhWTbcWpL6XII1RDIz+XRVx0IHJ8nZLyxbcxqlGEGYbR33z1bqw4O+4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicTemplate");
        // can help in debugging; otherwise not necessary
        relicTrackables.activate(); // Activate Vuforia


        double tX = 0; // X value extracted from our the offset of the traget relative to the robot.
        double tZ = 0; // Same as above but for Z
        double tY = 0; // Same as above but for Y
        // -----------------------------------
        double rX = 0; // X value extracted from the rotational components of the tartget relitive to the robot
        double rY = 0; // Same as above but for Y
        double rZ = 0; // Same as above but for Z


        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();


                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                    telemetry.addData("VuMark is", "Left");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    return 'l';
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    return 'r';
                } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    return 'c';
                }

            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }

        return '0';

    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void Rotire270(double speed, int counts){
        if (!opModeIsActive()) return;

        int newLeftTarget = 0;
        int newRightTarget = 0;

        robot.roatadr.setDirection(DcMotor.Direction.REVERSE);
        robot.roatast.setDirection(DcMotor.Direction.REVERSE);
        // Ensure that the opmode is still active
        if (opModeIsActive() ) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.roatast.getCurrentPosition() + counts;
            newRightTarget = robot.roatadr.getCurrentPosition() + counts;
            robot.roatast.setTargetPosition(newLeftTarget);
            robot.roatadr.setTargetPosition(newRightTarget);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.roatast.setPower(speed);
            robot.roatadr.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.roatast.isBusy() || robot.roatadr.isBusy())
                    && (robot.roatast.getCurrentPosition() <= newLeftTarget
                    || robot.roatadr.getCurrentPosition() <= newRightTarget)) {

                // Display it for the driver.
                telemetry.addData("Acum", "Merg spre %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("ticks-urile curente", "la %7d :%7d",
                        robot.roatast.getCurrentPosition(),
                        robot.roatadr.getCurrentPosition());
                telemetry.update();
            }

            robot.roatast.setPower(0);
            robot.roatadr.setPower(0);
            robot.roatast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.roatast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void Rotire90(double speed,int counts){              //ROTIRE -90;
        if (!opModeIsActive()) return;

        int newLeftTarget = 0;
        int newRightTarget = 0;

        robot.roatadr.setDirection(DcMotor.Direction.FORWARD);
        robot.roatast.setDirection(DcMotor.Direction.FORWARD);
        // Ensure that the opmode is still active
        if (opModeIsActive() ) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.roatast.getCurrentPosition() + counts;
            newRightTarget = robot.roatadr.getCurrentPosition() + counts;
            robot.roatast.setTargetPosition(newLeftTarget);
            robot.roatadr.setTargetPosition(newRightTarget);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.roatast.setPower(speed);
            robot.roatadr.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.roatast.isBusy() || robot.roatadr.isBusy())
                    && (robot.roatast.getCurrentPosition() <= newLeftTarget
                    || robot.roatadr.getCurrentPosition() <= newRightTarget)) {

                // Display it for the driver.
                telemetry.addData("Acum", "Merg spre %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("ticks-urile curente", "la %7d :%7d",
                        robot.roatast.getCurrentPosition(),
                        robot.roatadr.getCurrentPosition());
                telemetry.update();
            }

            robot.roatast.setPower(0);
            robot.roatadr.setPower(0);
            robot.roatast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.roatast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.roatadr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void DriveWithInsurance(){
        if (opModeIsActive()){
            robot.roatast.setPower(0.5);
            robot.roatadr.setPower(0.5);

            while (opModeIsActive() &&
                    (robot.roatast.isBusy() || robot.roatadr.isBusy())){

            }
        }
    }

    public void basculare (double pozitie1) {
        if (!opModeIsActive()) return;

        robot.tavaServo.setPosition(pozitie1);

        if (opModeIsActive()) {

            // Display it for the driver.
            telemetry.addData("Basculat", "E la pozitia", robot.tavaServo.getPosition());
            telemetry.update();

        }
    }

    public void savinacuburile(double timeoutS){
        robot.cubst.setPower(2);
        robot.cubdr.setPower(2);



        while(opModeIsActive() && (runtime.seconds() < timeoutS)){
            telemetry.addData("merge", "");
        }
    }

    public void lasaplaca(){
        robot.tavaServo.setPosition(0.25);
    }


    public void JewelsGo(double speed, int counts, int counts2, int counts3, int counts4) {
        if (!opModeIsActive()) return;
        int Target1 = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            Target1 = robot.mingiMotor.getCurrentPosition() + counts;

            robot.mingiMotor.setTargetPosition(Target1);


            // reset the timeout time and start motion.

            robot.mingiMotor.setPower(speed);


            while (opModeIsActive() &&
                    (robot.mingiMotor.isBusy())
                    && (robot.mingiMotor.getCurrentPosition() <= Target1)) {


                // Display it for the driver.
                telemetry.addData("merg spre", "%7d", Target1);
                telemetry.addData("sunt la pozitia", "%7d",
                        robot.mingiMotor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.mingiMotor.setPower(0);

            robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);


            robot.mingiServo.setPosition(0.25);

            sleep(250);



            int Target2 = 0;

            Target2 = robot.mingiMotor.getCurrentPosition() + counts2;

            robot.mingiMotor.setTargetPosition(Target2);


            // reset the timeout time and start motion.

            robot.mingiMotor.setPower(speed);


            while (opModeIsActive() &&
                    (robot.mingiMotor.isBusy())
                    && (robot.mingiMotor.getCurrentPosition() <= Target2)) {


                // Display it for the driver.
                telemetry.addData("merg spre", "%7d", Target2);
                telemetry.addData("sunt la pozitia", "%7d",
                        robot.mingiMotor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.mingiMotor.setPower(0);

            robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(1000);


            if (color_sensor1.isBlue() || color_sensor3.isRed()) {
                telemetry.addData("ROSU", "Vad rosu daram albastru");
                telemetry.update();
                if (!opModeIsActive()) return;
                robot.mingiServo.setPosition(0);
                sleep(500);
                if (!opModeIsActive()) return;
                int Target3 = 0;

                robot.mingiMotor.setDirection(DcMotor.Direction.FORWARD);
                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    Target3 = robot.mingiMotor.getCurrentPosition() + counts3;

                    robot.mingiMotor.setTargetPosition(Target3);


                    // reset the timeout time and start motion.

                    robot.mingiMotor.setPower(speed);


                    while (opModeIsActive() &&
                            (robot.mingiMotor.isBusy())
                            && (robot.mingiMotor.getCurrentPosition() <= Target3)) {


                        // Display it for the driver.
                        telemetry.addData("merg spre", "%7d", Target3);
                        telemetry.addData("sunt la pozitia", "%7d",
                                robot.mingiMotor.getCurrentPosition());
                        telemetry.update();
                    }


                    // Stop all motion;
                    robot.mingiMotor.setPower(0);

                    robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                }
                sleep(500);
                if (!opModeIsActive()) return;
                robot.mingiServo.setPosition(1);
                sleep(500);
                if (!opModeIsActive()) return;
                int Target4 = 0;

                robot.mingiMotor.setDirection(DcMotor.Direction.FORWARD);
                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    Target4 = robot.mingiMotor.getCurrentPosition() + counts4;

                    robot.mingiMotor.setTargetPosition(Target4);


                    // reset the timeout time and start motion.

                    robot.mingiMotor.setPower(speed);


                    while (opModeIsActive() &&
                            (robot.mingiMotor.isBusy())
                            && (robot.mingiMotor.getCurrentPosition() <= Target4)) {


                        // Display it for the driver.
                        telemetry.addData("merg spre", "%7d", Target4);
                        telemetry.addData("sunt la pozitia", "%7d",
                                robot.mingiMotor.getCurrentPosition());
                        telemetry.update();
                    }


                    // Stop all motion;
                    robot.mingiMotor.setPower(0);

                    robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                }
            } else if (color_sensor1.isRed() || color_sensor3.isBlue()) {
                telemetry.addData("ALBASTRU ", "Vad albastru daram albastru");
                telemetry.update();
                if (!opModeIsActive()) return;
                robot.mingiServo.setPosition(1);
                sleep(500);
                if (!opModeIsActive()) return;
                int Target3 = 0;

                robot.mingiMotor.setDirection(DcMotor.Direction.FORWARD);
                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    Target3 = robot.mingiMotor.getCurrentPosition() + counts3;

                    robot.mingiMotor.setTargetPosition(Target3);


                    // reset the timeout time and start motion.

                    robot.mingiMotor.setPower(speed);


                    while (opModeIsActive() &&
                            (robot.mingiMotor.isBusy())
                            && (robot.mingiMotor.getCurrentPosition() <= Target3)) {


                        // Display it for the driver.
                        telemetry.addData("merg spre", "%7d", Target3);
                        telemetry.addData("sunt la pozitia", "%7d",
                                robot.mingiMotor.getCurrentPosition());
                        telemetry.update();
                    }


                    // Stop all motion;
                    robot.mingiMotor.setPower(0);

                    robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                }
                sleep(500);
                if (!opModeIsActive()) return;
                robot.mingiServo.setPosition(0);
                if (!opModeIsActive()) return;
                int Target4 = 0;

                robot.mingiMotor.setDirection(DcMotor.Direction.FORWARD);
                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    Target3 = robot.mingiMotor.getCurrentPosition() + counts4;

                    robot.mingiMotor.setTargetPosition(Target3);


                    // reset the timeout time and start motion.

                    robot.mingiMotor.setPower(speed);


                    while (opModeIsActive() &&
                            (robot.mingiMotor.isBusy())
                            && (robot.mingiMotor.getCurrentPosition() <= Target4)) {


                        // Display it for the driver.
                        telemetry.addData("merg spre", "%7d", Target4);
                        telemetry.addData("sunt la pozitia", "%7d",
                                robot.mingiMotor.getCurrentPosition());
                        telemetry.update();
                    }


                    // Stop all motion;
                    robot.mingiMotor.setPower(0);

                    robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                }
            } else {
                if (!opModeIsActive()) return;
                int Target3 = 0;

                robot.mingiMotor.setDirection(DcMotor.Direction.FORWARD);
                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    Target3 = robot.mingiMotor.getCurrentPosition() + counts3;

                    robot.mingiMotor.setTargetPosition(Target3);


                    // reset the timeout time and start motion.

                    robot.mingiMotor.setPower(speed);


                    while (opModeIsActive() &&
                            (robot.mingiMotor.isBusy())
                            && (robot.mingiMotor.getCurrentPosition() <= Target3)) {


                        // Display it for the driver.
                        telemetry.addData("merg spre", "%7d", Target3);
                        telemetry.addData("sunt la pozitia", "%7d",
                                robot.mingiMotor.getCurrentPosition());
                        telemetry.update();
                    }


                    // Stop all motion;
                    robot.mingiMotor.setPower(0);

                    robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                }
                sleep(500);
                if (!opModeIsActive()) return;
                robot.mingiServo.setPosition(1);
                sleep(500);
                if (!opModeIsActive()) return;
                int Target4 = 0;

                robot.mingiMotor.setDirection(DcMotor.Direction.FORWARD);
                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    Target4 = robot.mingiMotor.getCurrentPosition() + counts4;

                    robot.mingiMotor.setTargetPosition(Target4);


                    // reset the timeout time and start motion.

                    robot.mingiMotor.setPower(speed);


                    while (opModeIsActive() &&
                            (robot.mingiMotor.isBusy())
                            && (robot.mingiMotor.getCurrentPosition() <= Target4)) {


                        // Display it for the driver.
                        telemetry.addData("merg spre", "%7d", Target4);
                        telemetry.addData("sunt la pozitia", "%7d",
                                robot.mingiMotor.getCurrentPosition());
                        telemetry.update();
                    }


                    // Stop all motion;
                    robot.mingiMotor.setPower(0);

                    robot.mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                }
            }
        }


    }

}