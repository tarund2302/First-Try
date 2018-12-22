package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFinder;

@Autonomous(name = "DemoAuto")
public class DemoAuto extends LinearOpMode implements Constants,AutonomousOpMode {

    private Hardware robot = new Hardware();
    private Drivetrain drivetrain = new Drivetrain(robot);

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
        robot.init(hardwareMap);

        GoldFinder gold = new GoldFinder(this, robot);
        gold.setAlignSettings(ALIGN_POSITION,1000);
        double goldPos = 0;

        waitForStart();

        gold.startOpenCV(hardwareMap);
        drivetrain.stop();
        sleep(5000);
        while(getOpModeIsActive() && !gold.isFound()){
            drivetrain.rotate(-0.27);
            telemetry.addData("Aligned:", gold.getAligned());
            telemetry.addData("Pos:",gold.getXPosition());
            telemetry.update();
        }
        sleep(500);
        drivetrain.stop();
        gold.alignGold();

        if(!getOpModeIsActive()){
            gold.disable();
        }
    }
}
