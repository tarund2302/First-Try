package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "ServoTesting")
public class ServoPositioning extends OpMode{

    public Servo servo;
    //double currPosition = servo.getPosition();

    @Override
    public void init() {
        //servo = hardwareMap.servo.get("servo");
        //servo.setPosition(0);
    }

    @Override
    public void loop() {
        double currPosition = servo.getPosition();

        if(gamepad1.dpad_right){
            servo.setPosition(currPosition + .1);
        }
        else if(gamepad1.dpad_left){
            servo.setPosition(currPosition - .1);
        }

        if(gamepad1.dpad_up){
            servo.setPosition(currPosition + .01);
        }
        else if(gamepad1.dpad_down){
            servo.setPosition(currPosition - .01);
        }
        telemetry.addData("Servo position:", servo.getPosition());
        telemetry.update();
    }
}