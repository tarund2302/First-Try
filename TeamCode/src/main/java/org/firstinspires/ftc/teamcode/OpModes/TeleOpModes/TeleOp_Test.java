package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

//v3 teleop code made by Tarun Dasari, 3846 Maelstrom Rookie Programmer
//last update: November 17, 2018

@TeleOp(name ="TeleOp_Test")
public class TeleOp_Test extends OpMode implements Constants {

    private Hardware robot = new Hardware();
    private Drivetrain drivetrain = new Drivetrain(hardwareMap);


    @Override
    public void init() {
        //initialize hardware
        robot.init(hardwareMap);

        //initialize servo positions
        robot.hookServo.setPosition(HOOK_UP_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);

        //reverse left side of the drivetrain
        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //controls for drivetrain
        drivetrain.leftDrive(gamepad1.left_stick_y);
        drivetrain.rightDrive(gamepad1.right_stick_y);



        //controls for extendo

        //extends
        robot.motorExtendo1.setPower(gamepad1.right_trigger);
        robot.motorExtendo2.setPower(gamepad1.right_trigger);

        //retracts
        robot.motorExtendo1.setPower(-gamepad1.left_trigger);
        robot.motorExtendo2.setPower(-gamepad1.left_trigger);


        //controls for actuator
        if(gamepad1.right_bumper)
        {
            robot.motorActuator.setPower(1); //extends
        }

        else
        {
            robot.motorActuator.setPower(0); //stops motor
        }

        if(gamepad1.left_bumper)
        {
            robot.motorActuator.setPower(-1); //retracts
        }

        else
        {
            robot.motorActuator.setPower(0); //stops motor
        }


        //hook controls
        if(gamepad1.a) //hook down
        {
            robot.hookServo.setPosition(HOOK_DOWN_POSITION);
        }

        if(gamepad1.x) //hook up
        {
            robot.hookServo.setPosition(HOOK_UP_POSITION);
        }

        //invert drivetrain controls
        if(gamepad1.y)
        {
            drivetrain.leftDrive(-gamepad1.left_stick_y);
            drivetrain.rightDrive(-gamepad1.left_stick_y);
        }

        else //leave controls as normal
        {
            drivetrain.leftDrive(gamepad1.left_stick_y);
            drivetrain.rightDrive(gamepad1.left_stick_y);
        }

        // Show the speed of drivetrain motors
        telemetry.addData("Left Front Motor Speed: ", robot.motorFrontLeft.getPower());
        telemetry.addData("Right Front Motor Speed: ", robot.motorFrontRight.getPower());
        telemetry.addData("Left Back Motor Speed: ", robot.motorBackLeft.getPower());
        telemetry.addData("Right Back Motor Speed: ", robot.motorBackRight.getPower());
        telemetry.update();

    }
}
