package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")

public class PushbotTeleopPOV_Linear extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();
    boolean a = true;
    boolean c = true;
    double puteref=0;
    double puteres=0;
    double putereCamera = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            puteref = 0;
            puteres = 0;

            if (gamepad1.x) {
                putereCamera = 1;
            } else if (gamepad1.a) {
                putereCamera = -1;
            } else {
                putereCamera = 0;
            }
            robot.motorCamera.setPower(putereCamera);
            if (gamepad1.left_bumper) {
                robot.roatadr.setPower(-gamepad1.right_stick_y * 0.3);
                robot.roatast.setPower(-gamepad1.left_stick_y * 0.3);
            } else {
                robot.roatadr.setPower(-gamepad1.right_stick_y);
                robot.roatast.setPower(-gamepad1.left_stick_y);
            }

        }

    }
}