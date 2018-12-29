package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

//v3 teleop code made by Tarun Dasari, Rookie Programmer of 3846 Maelstrom
//last update: December 18, 2018



@TeleOp(name ="TeleOp_Test")
public class TeleOp_Test extends OpMode implements Constants {

    private Hardware robot = new Hardware();
    private Toggle toggle = new Toggle();
    private double xDirection;
    private double yDirection;
    DcMotor winch;
    /*private boolean leftStickButton = toggle.toggle(gamepad1.left_stick_button);
    private boolean rightStickButton = toggle.toggle(gamepad1.right_stick_button);*/
    //double currDistance = robot.rangeSensor.getDistance(DistanceUnit.INCH);
    @Override
    public void init() {
        //initialize hardware
        robot.init(hardwareMap);


    }

    @Override
    public void loop() {
        yDirection = SPEED_MUlTIPLIER * gamepad1.left_stick_y;
        xDirection = SPEED_MUlTIPLIER * gamepad1.right_stick_x;

        double y = yDirection - xDirection;
        double x = yDirection + xDirection;

        robot.drivetrain.leftDrive(y);
        robot.drivetrain.rightDrive(x);

        if(gamepad1.a){
            robot.winch.setPower(1);
        }
        else if (gamepad1.b){robot.winch.setPower(-1);}
        else{robot.winch.setPower(0);}

        //telemetry
        telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
        telemetry.addData("Left Front Motor Speed: ", robot.motorFrontLeft.getPower());
        telemetry.addData("Right Front Motor Speed: ", robot.motorFrontRight.getPower());
        telemetry.addData("Left Back Motor Speed: ", robot.motorBackLeft.getPower());
        telemetry.addData("Right Back Motor Speed: ", robot.motorBackRight.getPower());
        //telemetry.addData("Range distance:", currDistance);
        telemetry.update();
    }
}
