package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public interface Constants {

    //Hardware HardwareMap;

    //public AutonomousOpMode auto;

    //public Telemetry telemetry;

    //public Drivetrain drivetrain;

    //hook positions
    double HOOK_UP_POSITION = 0;
    double HOOK_DOWN_POSITION = 1;

    //marker positions
    double MARKER_UP_POSITION = 1;
    double MARKER_DOWN_POSITION = -1;

    //latch drop positions
    double DROP_UP_POSITION = 1;
    double DROP_DOWN_POSITION = -1;

    //extendo power values
    double EXTENDO_EXTEND_POWER = 0.6;
    double EXTENDO_RETRACT_POWER = -0.6;

    double NEVEREST_20_COUNTS_PER_REV = 560; //extendo motor ticks
    double NEVEREST_40_COUNTS_PER_REV = 1120; //drivetrain motor ticks
    double ORBITAL_3_7_COUNTS_PER_REV = 103; //actuator motor ticks


    double WHEEL_DIAMETER_INCHES = 4.0;
    double WHEEL_CIRCUM = WHEEL_DIAMETER_INCHES * Math.PI;
    double DRIVE_GEAR_REDUCTION = 0.5; //gear ratio (driven gear / driving gear)
    double DRIVEN_GEAR_REDUCTION = 2.0; //gear ratio (driving gear / driven gear)
    double COUNTS_PER_INCH = (NEVEREST_40_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUM;

    double DISTANCE_TOLERANCE = .5;
    double IMU_TOLERANCE = 0.5;

    //used for encoder drive
    double DRIVE_SPEED = 0.5;
    double TURN_SPEED = 0.2;

    double NANOSECS_PER_MIN = 6e+10;
    long NANOSECS_PER_MILISEC = 1000000;

    //PID will be tested
    double dtKP = 0.005;
    double dtKI = 0;
    double dtKD = 0;
    double dtMaxI = 1;

    double turnKP = 0.01;
    double turnKI = 0;
    double turnKD = 0;
    double turnMaxI = 1;

    double turnBigKP = 0.01;
    double turnBigKI = 0;
    double turnBigKD = 0;
    double turnBigMaxI = 1;

    double alignGoldKP = 0.001;
    double alignGoldKI = 0;
    double alignGoldKD = 0;
    double alignGoldMaxI = 0;

    int ALIGN_POSITION = -100;


}
