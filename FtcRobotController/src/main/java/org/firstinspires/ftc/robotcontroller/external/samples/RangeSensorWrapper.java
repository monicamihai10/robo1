package org.firstinspires.ftc.robotcontroller.external.samples;

/*
Modern Robotics Range Sensors Example
Created 10/31/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "range28" (MRI Range Sensor with default I2C address 0x28
I2CDevice "range2a" (MRI Color Sensor with I2C address 0x2a

ModernRoboticsI2cGyro is not being used because it does not support .setI2CAddr().

To change range sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import static com.qualcomm.robotcore.hardware.I2cAddr.create8bit;


public class RangeSensorWrapper {

    byte[] rangeCache;

    private Telemetry telemetry;
    private I2cDevice rangeSensor;
    private I2cDeviceSynch rangeReader;



    public RangeSensorWrapper(final String sensorName, final HardwareMap hardwareMap, I2cAddr sensorAddress, final Telemetry telemetry){

        this.rangeSensor = hardwareMap.i2cDevice.get(sensorName);

        this.rangeReader = new I2cDeviceSynchImpl(rangeSensor, sensorAddress, false);
        this.rangeReader.engage();

        this.telemetry = telemetry;

    }

    public double getDistanceInCm(){
        rangeCache = rangeReader.read(0x04, 2);
        double optic = rangeCache[1] & 0xFF;
        double sonic = rangeCache[0] & 0xFF;


        return sonic;
        //   telemetry.addData("ultrasonic",sonic);
        //   telemetry.addData("optic",optic);
        //   telemetry.update();

     /*
        if (optic>0){
            if (optic>150){
                return 2;
            }else if(optic>70){
                return 3;
            } else if(optic>25){
                return 4;
            }else {
                return sonic;
            }
        }else{
            return sonic;
        }
*/


        /*
        altitude
            back (cm)   reading
            2           250
            2.5         154
            3           90
            3.5         63
            4           49
            4.5         32
            5           26
            6           16
            7           10
            8           7
            9           5
            10          4
        */



  /*
        double dist = rangeCache[0] & 0xFF;
        if(dist<=6) {
            return rangeCache[1] & 0xFF;
            //return  getDistanceInCmOptical();
        } else{
            return dist;
        }
*/
        //return rangeCache[0] & 0xFF;
    }

    public double getDistanceInCmOptical(){
        rangeCache = rangeReader.read(0x04, 2);
        return rangeCache[1] & 0xFF;
    }



}