package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

//v3 teleop code made by Tarun Dasari, Rookie Programmer of 3846 Maelstrom
//last update: November 21, 2018

@TeleOp(name ="TeleOp_Test")
public class TeleOp_Test extends OpMode implements Constants {

    private Hardware robot = new Hardware();

    //private Drivetrain drivetrain = new Drivetrain(Hardware hardwareMap);

    boolean dPadRightCurrState = false;
    boolean dPadRightPreviousState = false;
    boolean invertDrive = true;


    @Override
    public void init() {
        //initialize hardware
        robot.init(hardwareMap);

        //initialize servo positions
        robot.hookServo.setPosition(HOOK_UP_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);



    }

    @Override
    public void loop() {

        //controls for drivetrain
        robot.drivetrain.leftDrive(gamepad1.left_stick_y);
        robot.drivetrain.rightDrive(gamepad1.right_stick_y);

        //controls for extendo

        //extends
        if(gamepad1.right_trigger > 0)
        {
            robot.motorExtendo1.setPower(EXTENDO_EXTEND_POWER);
            robot.motorExtendo2.setPower(EXTENDO_EXTEND_POWER);
        }

        //retracts
        else if(gamepad1.left_trigger > 0)
        {
            robot.motorExtendo1.setPower(EXTENDO_RETRACT_POWER);
            robot.motorExtendo2.setPower(EXTENDO_RETRACT_POWER);
        }

        else
        {
            robot.motorExtendo1.setPower(0);
            robot.motorExtendo2.setPower(0);
        }



        //controls for actuator
        if(gamepad1.right_bumper)
        {
            robot.motorActuator.setPower(1); //extends
        }


        else if(gamepad1.left_bumper)
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
            robot.markerServo.setPosition(MARKER_DOWN_POSITION);
        }

        if(gamepad1.x) //hook up
        {
            robot.markerServo.setPosition(MARKER_UP_POSITION);
        }


        //toggling for inverted controls
        if(gamepad1.dpad_right)
        {
            dPadRightCurrState = true;
        }

        else
        {
            dPadRightCurrState = false;
            if (dPadRightPreviousState)
            {
                invertDrive = !invertDrive;
            }
        }

        dPadRightPreviousState = dPadRightCurrState;

        //invert drivetrain controls
        if(invertDrive) //invert the drivetrain
        {
            robot.drivetrain.leftDrive(-gamepad1.left_stick_y);
            robot.drivetrain.rightDrive(-gamepad1.left_stick_y);
        }

        else //leave controls as normal
        {
            robot.drivetrain.leftDrive(gamepad1.left_stick_y);
            robot.drivetrain.rightDrive(gamepad1.left_stick_y);
        }

        // Show the speed of drivetrain motors
        telemetry.addData("Left Front Motor Speed: ", robot.motorFrontLeft.getPower());
        telemetry.addData("Right Front Motor Speed: ", robot.motorFrontRight.getPower());
        telemetry.addData("Left Back Motor Speed: ", robot.motorBackLeft.getPower());
        telemetry.addData("Right Back Motor Speed: ", robot.motorBackRight.getPower());
        telemetry.update();

    }
}
