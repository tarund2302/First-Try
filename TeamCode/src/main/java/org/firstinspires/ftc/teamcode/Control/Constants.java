package org.firstinspires.ftc.teamcode.Control;

public interface Constants {

    //hook positions
    double HOOK_UP_POSITION = 1;
    double HOOK_DOWN_POSITION = 0;

    //marker positions
    double MARKER_UP_POSITION = 0;
    double MARKER_DOWN_POSITION = 0.7;

    double NEVEREST_20_COUNTS_PER_REV = 560; //extendo motors
    double NEVEREST_40_COUNTS_PER_REV = 1120; //drivetrain motors
    double ORBITAL_3_7_COUNTS_PER_REV = 103; //actuator motors

    double WHEEL_DIAMETER_INCHES = 4.0;
    double DRIVE_GEAR_REDUCTION = 2.0; //gear ratio
    double COUNTS_PER_INCH = (NEVEREST_40_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);




}
