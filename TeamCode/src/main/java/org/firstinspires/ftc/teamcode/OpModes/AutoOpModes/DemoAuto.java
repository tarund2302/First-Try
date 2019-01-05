package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Direction;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.GoldFinder;

@Autonomous(name = "DemoAuto")
public class DemoAuto extends LinearOpMode implements Constants,AutonomousOpMode {

    private Hardware robot = new Hardware();

    public boolean getOpModeIsActive(){
        return opModeIsActive();
    }
    public Telemetry getTelemetry(){
        return telemetry;
    }

    public void runOpMode(){
        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);

        robot.gold.setAlignSettings(ALIGN_POSITION,1000);
        double goldPos = 0;

        waitForStart();

        robot.gold.startOpenCV(hardwareMap);
        sleep(3000);
        /*while(getOpModeIsActive() && !robot.gold.isFound()){
            robot.drivetrain.rotate(-0.33);
            telemetry.addData("Aligned:", robot.gold.getAligned());
            telemetry.addData("Pos:",robot.gold.getXPosition());
            telemetry.update();
        }*/
        //robot.drivetrain.rotateForTime(0.5,500,Direction.LEFT);
        while(getOpModeIsActive() && !robot.gold.isFound()){
            robot.drivetrain.rotate(0.33, Direction.RIGHT);
            telemetry.addData("Found:", robot.gold.isFound());
            telemetry.addData("Pos:",robot.gold.getXPosition());
            telemetry.update();
        }
        //robot.gold.goldSearch(0.33,Direction.RIGHT);
        sleep(500);
        robot.drivetrain.stop();
        robot.gold.demoAlignGold();

    }
}
