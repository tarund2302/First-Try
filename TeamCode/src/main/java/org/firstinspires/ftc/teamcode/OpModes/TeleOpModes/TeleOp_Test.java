package org.firstinspires.ftc.teamcode.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name ="TeleOp_Test")
public class TeleOp_Test extends OpMode implements Constants {

    private Hardware robot = new Hardware();
    private Toggle toggle = new Toggle(gamepad1);
    private double yDirection;
    private double xDirection;

    public void init() {
        robot.init(hardwareMap);
    }

    public void loop() {
/*        yDirection = SPEED_MUlTIPLIER * gamepad1.left_stick_y;
        xDirection = SPEED_MUlTIPLIER * gamepad1.right_stick_x;

        double left = yDirection - xDirection;
        double right = yDirection + xDirection;

        robot.drivetrain.leftDrive(left);
        robot.drivetrain.rightDrive(right);*/

        robot.drivetrain.driveNFS(gamepad1);
       robot.climber.driverControl(gamepad1);

            if(gamepad1.dpad_up){robot.climber.autoRaise();}
            else if(gamepad1.dpad_down){robot.climber.autoLower();}
            else {robot.climber.liftStop();}

        //telemetry
        telemetry.addData("Left Front Speed: ", robot.motorFrontLeft.getPower());
        telemetry.addData("Right Front Speed: ", robot.motorFrontRight.getPower());
        telemetry.addData("Left Back Speed: ", robot.motorBackLeft.getPower());
        telemetry.addData("Right Back Speed: ", robot.motorBackRight.getPower());
        telemetry.addData("Latch Power: ", robot.latch.getPower());
        telemetry.addData("Hang Position: ", robot.hangMotor.getCurrentPosition());
        //telemetry.addData("Range distance:", robot.rangeSensor.getData());
        telemetry.update();
    }
}
