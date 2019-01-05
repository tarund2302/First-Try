package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Subsystems.AutoPaths.CraterPaths;
import org.firstinspires.ftc.teamcode.Subsystems.AutoPaths.DepotPaths;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.ChainLift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotComponents.GoldFinder;

public class Hardware implements Constants {

    HardwareMap hardwareMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public BNO055_IMU imu;

    //public MaxbotixUltrasonicSensor rangeSensor;

    public CRServo latch;

    //drivetrain
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor hangMotor;

    public Drivetrain drivetrain;
    public GoldFinder gold;
    public ChainLift climber;
    public DepotPaths dPaths;
    public CraterPaths cPaths;

    public DcMotor[] drivetrainMotors;

    public void init (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        imu = new BNO055_IMU("imu", this);
        /*  rangeSensor = new MaxbotixUltrasonicSensor(hardwareMap.analogInput.get("rangeSensor"));*/

        //initialize drivetrain
        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        motorBackRight = hardwareMap.dcMotor.get("BackRight");
        hangMotor = hardwareMap.dcMotor.get("hang");

       DcMotor[] tempMotorArray = {motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};
       drivetrainMotors = tempMotorArray;

        //reverse left side of drivetrain
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        latch = hardwareMap.crservo.get("latch");

        drivetrain = new Drivetrain(this);
        climber = new ChainLift(this);
        gold = new GoldFinder(this);
        dPaths = new DepotPaths(this);
        cPaths = new CraterPaths(this);
    }

    public void setAuto(AutonomousOpMode auto){
        this.auto = auto;
    }

    public void setTelemetry (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap()
    {
        return hardwareMap;
    }

}
