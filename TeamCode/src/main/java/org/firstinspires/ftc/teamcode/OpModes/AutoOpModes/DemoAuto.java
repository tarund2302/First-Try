package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFinder;

@Autonomous(name = "DemoAuto")
public class DemoAuto extends LinearOpMode implements Constants,AutonomousOpMode {

    private Hardware robot = new Hardware();

    public boolean getOpModeIsActive()
    {
        return opModeIsActive();
    }
    public Telemetry getTelemetry()
    {
        return telemetry;
    }

    @Override
    public void runOpMode(){
        robot.setAuto(this);
        robot.setTelemetry(telemetry);

        GoldFinder gold = new GoldFinder(this, robot);
        gold.setAlignSettings(ALIGN_POSITION,1000);
        robot.init(hardwareMap);
        double goldPos = 0;

        waitForStart();

        gold.startOpenCV(hardwareMap);

        while(getOpModeIsActive() && !gold.isFound()){
            robot.drivetrain.rotate(-0.30);
            telemetry.addData("Aligned:", gold.getAligned());
            telemetry.addData("Pos:",gold.getXPosition());
            telemetry.update();
        }
        sleep(1000);
        robot.drivetrain.stop();
        gold.alignGold();



    }
}
