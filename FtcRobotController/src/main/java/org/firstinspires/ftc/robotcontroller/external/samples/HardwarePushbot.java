/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot {

    public DcMotor  roatast     =null;          //ROTI
    public DcMotor  roatadr     =null;




    public DcMotor  cubst       = null;         //CUB
    public DcMotor  cubdr       = null;
    public Servo    block       = null;



    public DcMotor  tavaMotor      = null;         //TAVA
    public Servo    tavaServo      = null;


    public DcMotor  mingiMotor      =null;
    public Servo    mingiServo      =null;

    public DcMotor  Relicva         =null;
    public Servo    wrist           =null;
    public Servo    deget           =null;




    //* local OpMode members.
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();





    //* Constructor
    public HardwarePushbot(){

    }

    //* Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {




        // HardwareMap
        hwMap = ahwMap;



        roatadr     =hwMap.get(DcMotor.class, "roata_dreapta");
        roatast     =hwMap.get(DcMotor.class, "roata_stanga");
        cubdr       =hwMap.get(DcMotor.class, "cubul_dreapta");
        cubst       =hwMap.get(DcMotor.class, "cubul_stanga");
        tavaMotor   =hwMap.get(DcMotor.class, "tavaMotor");
        tavaServo   =hwMap.get(Servo.class, "tavaServo");
        mingiMotor  =hwMap.get(DcMotor.class, "mingiM");
        mingiServo  =hwMap.get(Servo.class,"mingiS");
        block       =hwMap.get(Servo.class,"blocheaza");
        Relicva     =hwMap.get(DcMotor.class,"relicva");
        wrist       =hwMap.get(Servo.class, "articulatie");
        deget       =hwMap.get(Servo.class, "deg");

        //POZITII

        tavaServo.setPosition(0.25);
        mingiServo.setPosition(1);
        block.setPosition(0);
        wrist.setPosition(0);
        deget.setPosition(0.80);


        //DIRECTIE
        roatadr.setDirection(DcMotor.Direction.REVERSE);
        roatast.setDirection(DcMotor.Direction.FORWARD);
        cubdr.setDirection(DcMotor.Direction.REVERSE);
        cubst.setDirection(DcMotor.Direction.FORWARD);
        tavaMotor.setDirection(DcMotor.Direction.FORWARD);
        mingiMotor.setDirection(DcMotor.Direction.REVERSE);
        Relicva.setDirection(DcMotor.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        //ENCODERE
        roatadr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roatast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tavaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mingiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cubst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cubdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Relicva.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roatadr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roatast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tavaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mingiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }





}