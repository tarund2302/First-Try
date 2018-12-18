package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

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

    }

    @Override
    public void loop() {
        yDirection = SPEED_MUlTIPLIER * gamepad1.left_stick_y;
        xDirection = SPEED_MUlTIPLIER * gamepad1.right_stick_x;

        double y = yDirection - xDirection;
        double x = yDirection + xDirection;

        // NFS controls for drivetrain
        robot.drivetrain.leftDrive(y);
        robot.drivetrain.rightDrive(x);

        //invert drivetrain controls
        if(toggle.toggle(gamepad1.y)) //switch to tank
        {
            if(toggle.toggle(gamepad1.x)) //invert the drivetrain
            {
                robot.drivetrain.leftDrive(SPEED_MUlTIPLIER * -gamepad1.left_stick_y);
                robot.drivetrain.rightDrive(SPEED_MUlTIPLIER * -gamepad1.right_stick_y);
                telemetry.addLine("Tank invert");
                telemetry.update();
            }
            else //leave controls as normal
            {
                robot.drivetrain.leftDrive(SPEED_MUlTIPLIER * gamepad1.left_stick_y);
                robot.drivetrain.rightDrive(SPEED_MUlTIPLIER * gamepad1.right_stick_y);
                telemetry.addLine("Tank normal");
                telemetry.update();

            }
        }
        else //do NFS controls
        {
            if(toggle.toggle(gamepad1.x)) //invert the drivetrain
            {
                robot.drivetrain.leftDrive(-y);
                robot.drivetrain.rightDrive(-x);
                telemetry.addLine("NFS invert");
                telemetry.update();

            }
            else //leave controls as normal
            {
                robot.drivetrain.leftDrive(y);
                robot.drivetrain.rightDrive(x);
                telemetry.addLine("NFS normal");
                telemetry.update();
            }
        }

        //controls for actuator
        if(gamepad1.right_bumper){
            robot.actuator.raise();
        }
        else if(gamepad1.left_bumper){
            robot.actuator.lower();
        }
        else{
            robot.actuator.stop();
        }

        //controls for extender
        if(gamepad1.right_trigger > 0){
            robot.markerSystem.extend();
        }
        else if(gamepad1.left_trigger > 0){
            robot.markerSystem.retract();
        }
        else{
            robot.markerSystem.stop();
        }

        //controls for marker
        if(toggle.toggle(gamepad1.dpad_right)){
            robot.markerSystem.drop();
        }
        else{
            robot.markerSystem.raise();
        }

        //telemetry
        //show the speed of drivetrain motors
        telemetry.addData("Left Front Motor Speed: ", robot.drivetrain.getData()[0]);
        telemetry.addData("Right Front Motor Speed: ", robot.drivetrain.getData()[1]);
        telemetry.addData("Left Back Motor Speed: ", robot.drivetrain.getData()[2]);
        telemetry.addData("Right Back Motor Speed: ", robot.drivetrain.getData()[3]);
        telemetry.addData("Actuator:",robot.actuator.getData());
        telemetry.update();

    }
}
