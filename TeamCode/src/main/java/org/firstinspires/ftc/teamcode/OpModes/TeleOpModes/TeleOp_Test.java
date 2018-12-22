package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private double yDirection;
    private double xDirection;
    double currDistance = robot.rangeSensor.getDistance(DistanceUnit.INCH);
    @Override
    public void init() {
        //initialize hardware
        robot.init(hardwareMap);
        //robot.climber.reset();

    }

    @Override
    public void loop() {
/*

        yDirection = SPEED_MUlTIPLIER * gamepad1.left_stick_y;
        xDirection = SPEED_MUlTIPLIER * gamepad1.right_stick_x;

        double y = yDirection - xDirection;
        double x = yDirection + xDirection;
*/
        // NFS controls for drivetrain
        /*robot.drivetrain.leftDrive(y);
        robot.drivetrain.rightDrive(x);*/
        robot.drivetrain.driveNFS(gamepad1.left_stick_y, gamepad1.right_stick_x);

        //invert drivetrain controls
        if(toggle.toggle(gamepad1.y)){ //switch to tank controls

            if(toggle.toggle(gamepad1.x)){ //invert the drivetrain
                /*robot.drivetrain.leftDrive(SPEED_MUlTIPLIER * -gamepad1.left_stick_y);
                robot.drivetrain.rightDrive(SPEED_MUlTIPLIER * -gamepad1.right_stick_y);*/
                robot.drivetrain.driveTank(-gamepad1.left_stick_y,-gamepad1.right_stick_y);
                telemetry.addLine("Tank invert");
                telemetry.update();
            }
            else{//leave controls as normal
                /*robot.drivetrain.leftDrive(SPEED_MUlTIPLIER * gamepad1.left_stick_y);
                robot.drivetrain.rightDrive(SPEED_MUlTIPLIER * gamepad1.right_stick_y);*/
                robot.drivetrain.driveTank(gamepad1.left_stick_y,gamepad1.right_stick_y);
                telemetry.addLine("Tank normal");
                telemetry.update();
            }
        }
        else{ //switch to NFS controls
            if(toggle.toggle(gamepad1.x)){//invert the drivetrain
                /*robot.drivetrain.leftDrive(-y);
                robot.drivetrain.rightDrive(-x);*/
                robot.drivetrain.driveNFS(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
                telemetry.addLine("NFS invert");
                telemetry.update();

            }
            else{ //leave controls as normal
                /*robot.drivetrain.leftDrive(y);
                robot.drivetrain.rightDrive(x);*/
                robot.drivetrain.driveNFS(gamepad1.left_stick_y, gamepad1.right_stick_x);
                telemetry.addLine("NFS normal");
                telemetry.update();
            }
        }
/*

        //controls for climber
        if(gamepad1.right_trigger > 0){
            robot.climber.raise();
        }
        else if(gamepad1.left_trigger > 0){
            robot.climber.lower();
        }
        else{
            robot.climber.stop();
        }
*/

        //telemetry
        telemetry.addData("Left Front Motor Speed: ", robot.motorFrontLeft.getPower());
        telemetry.addData("Right Front Motor Speed: ", robot.motorFrontRight.getPower());
        telemetry.addData("Left Back Motor Speed: ", robot.motorBackLeft.getPower());
        telemetry.addData("Right Back Motor Speed: ", robot.motorBackRight.getPower());
 /*       telemetry.addData("Actuator Power:",robot.climber.getData()[0]);
        telemetry.addData("Actuator Position:",robot.climber.getData()[1]);*/
        telemetry.addData("Range distance:", currDistance);
        telemetry.update();
    }
}
