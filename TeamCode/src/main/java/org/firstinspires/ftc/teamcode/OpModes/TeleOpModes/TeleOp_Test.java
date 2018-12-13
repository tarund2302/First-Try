package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

//v3 teleop code made by Tarun Dasari, Rookie Programmer of 3846 Maelstrom
//last update: December 9, 2018

@TeleOp(name ="TeleOp_Test")
public class TeleOp_Test extends OpMode implements Constants {

    private Hardware robot = new Hardware();
    private Toggle toggle = new Toggle();
    private double yDirection;
    private double xDirection;

    @Override
    public void init() {
        //initialize hardware
        robot.init(hardwareMap);

        //initialize servo positions
        /*robot.hookServo.setPosition(HOOK_UP_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);*/

    }

    @Override
    public void loop() {
        yDirection = 0.5 * gamepad1.left_stick_y;
        xDirection = 0.5 * gamepad1.right_stick_x;

        double y = yDirection - xDirection;
        double x = yDirection + xDirection;

        // NFS controls for drivetrain
/*        robot.drivetrain.leftDrive(y);
        robot.drivetrain.rightDrive(x);*/

        /*//controls for extendo

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

*/
        //toggle latch controls
/*        if(toggle.toggle(gamepad1.x)) //latch down
        {
            robot.latchServo.setPosition(DROP_UP_POSITION);
        }

        else //latch up
            {
            robot.latchServo.setPosition(DROP_DOWN_POSITION);
        }*/


        //invert drivetrain controls


        if(toggle.toggle(gamepad1.dpad_up)) //switch to tank
        {

            if(toggle.toggle(gamepad1.dpad_right)) //invert the drivetrain
            {
                robot.drivetrain.leftDrive(0.5 * -gamepad1.left_stick_y);
                robot.drivetrain.rightDrive(0.5 * -gamepad1.right_stick_y);
            }
            else //leave controls as normal
            {
                robot.drivetrain.leftDrive(0.5 * gamepad1.left_stick_y);
                robot.drivetrain.rightDrive(0.5 * gamepad1.right_stick_y);
            }

        }

        else //do NFS controls
        {
            if(toggle.toggle(gamepad1.dpad_right)) //invert the drivetrain
            {
                robot.drivetrain.leftDrive(-y);
                robot.drivetrain.rightDrive(-x);
            }
            else //leave controls as normal
            {
                robot.drivetrain.leftDrive(y);
                robot.drivetrain.rightDrive(x);
            }
        }

        // Show the speed of drivetrain motors
        telemetry.addData("Left Front Motor Speed: ", robot.motorFrontLeft.getPower());
        telemetry.addData("Right Front Motor Speed: ", robot.motorFrontRight.getPower());
        telemetry.addData("Left Back Motor Speed: ", robot.motorBackLeft.getPower());
        telemetry.addData("Right Back Motor Speed: ", robot.motorBackRight.getPower());
        telemetry.update();

    }
}
