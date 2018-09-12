package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")

public class PushbotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();
    double miscare = 1;
    boolean a = true;
    boolean b = true;
    boolean c = true;
    double puteref=0;
    double puteres=0;
    double Target = 0.85;
    float leftPower,rightPower,xValue,yValue;
    RangeSensorWrapper range_sensors;
    I2cAddr a1 = I2cAddr.create8bit(0x28);



    @Override
    public void runOpMode() {


        range_sensors = new RangeSensorWrapper("spate", hardwareMap, a1, telemetry);

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */// sc1 = hardwareMap.get(ColorSensor.class, "color_sensor1");
        //sc1.enableLed(true);
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            puteref=0;
            puteres=0;

            if (gamepad1.left_bumper) {
                robot.roatadr.setPower(-gamepad1.right_stick_y * 0.3);
                robot.roatast.setPower(-gamepad1.left_stick_y*0.3);
            } else {
                robot.roatadr.setPower(-gamepad1.right_stick_y);
                robot.roatast.setPower(-gamepad1.left_stick_y);
            }










            if(gamepad2.b){
                robot.tavaServo.setPosition(0.25);
            }

            if(gamepad2.a){
                robot.tavaServo.setPosition(1);
            }


            robot.Relicva.setPower(-gamepad2.right_stick_y);

            if (gamepad2.y){
                robot.wrist.setDirection(Servo.Direction.REVERSE);
                robot.wrist.setPosition(0.7);
            }

            if (gamepad2.x){
                robot.wrist.setDirection(Servo.Direction.REVERSE);
                robot.wrist.setPosition(0.1);
            }


            if (gamepad2.left_bumper){
                robot.deget.setPosition(0);
            }

            if (gamepad2.right_bumper){
                robot.deget.setPosition(0.80);
            }

/*
            if(gamepad2.dpad_right || (!a&& !c)){
                robot.cubst.setPower(-1);
                robot.cubdr.setPower(-1);
                a = false;
                c = false;
            }



            if(gamepad2.dpad_down || (a && b && c)) {
                robot.cubst.setPower(0);
                robot.cubdr.setPower(0);
                a = true;
                b = true;
                c = true;
            }

*/


       /*    puteref=gamepad2.left_trigger;
           puteres=gamepad2.right_trigger;
           if(puteref!=0 && puteres==0)
           {
               robot.cubst.setPower(puteref);
               robot.cubdr.setPower(puteref);

           }
           if(puteres!=0 && puteref==0)

           {
               robot.cubst.setPower(-puteres);
               robot.cubdr.setPower(-puteres);
           }


*/
            if(gamepad2.dpad_up){
                robot.cubst.setPower(1);
                robot.cubdr.setPower(1);
            }

            if (gamepad2.dpad_down){
                robot.cubst.setPower(-1);
                robot.cubdr.setPower(-1);
            }

            if (gamepad2.dpad_left) {
                robot.cubst.setPower(0);
                robot.cubdr.setPower(0);
            }
            robot.tavaMotor.setPower(gamepad2.left_stick_y);

            robot.mingiMotor.setPower(gamepad2.left_trigger);
            robot.mingiMotor.setPower(-gamepad2.right_trigger);
            robot.mingiServo.setPosition(miscare);

            if (gamepad1.a){
                robot.block.setPosition(0.25);
            }

            if (gamepad1.b){
                robot.block.setPosition(0);
            }



         /*   if(robot.tavaMotor.getCurrentPosition() < 2200){
                robot.tavaMotor.setPower(gamepad2.left_stick_y);
            } else if (robot.tavaMotor.getCurrentPosition() > 2200) {
                robot.tavaMotor.setPower(-gamepad2.left_stick_y);           //SWITCH DIRECTION TO AVOID EXITING THE RACK
            } else if (robot.tavaMotor.getCurrentPosition() < 0){
                robot.tavaMotor.setPower(-gamepad2.left_stick_y);
            }
*/



        }


    }}