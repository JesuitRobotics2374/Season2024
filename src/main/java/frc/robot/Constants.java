package frc.robot;

public class Constants {

    public static final int TEST_TARGET_TAG = 8;

    public static final String CAN_BUS_NAME_CANIVORE = "FastFD";
    public static final String CAN_BUS_NAME_ROBORIO = "rio";
    public static final String CAN_BUS_NAME_DRIVETRAIN = CAN_BUS_NAME_CANIVORE;

    // Contoller IDs
    public static final int CONTROLLER_USB_PORT_DRIVER = 0; // Drivers Controller
    public static final int CONTROLLER_USB_PORT_OPERATOR = 1; // Ordanence operators controller

    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // CHASSIS Subsystem
    public static final String TEST_ROBORIO_SERIAL_NUMBER = "0316b2d6"; // serial number of Swervee roborio

    // ARM Subsystem
    public static final int RIGHT_ARM_MOTOR_ID = 6;
    public static final int LEFT_ARM_MOTOR_ID = 7;
    public static final int ARM_ENCODER_ID = 25;
    public static final double FORWARD_SOFT_STOP = 0.15;
    public static final double BACKWARD_SOFT_STOP = -0.265;
    public static final double HOLD_NOTE = -0.19;

    // PID
    public static final double ARM_PID_P = 4.0;
    public static final double ARM_PID_I = 0.25;
    public static final double ARM_PID_D = 0.0;

    // Profiled PID
    public static final double P_ARM_PID_P = 2.0;
    public static final double P_ARM_PID_I = 0.05;
    public static final double P_ARM_PID_D = 0.1;

    // Center of field
    public static final double CENTER_FIELD_X = 8.4;

    // Translation2d
    public static final double RED_SPEAKER_X = 16.49;
    public static final double RED_SPEAKER_Y = 5.55;

    public static final double BLUE_SPEAKER_X = 0.05;
    public static final double BLUE_SPEAKER_Y = 5.55;

    // Align speeds
    public static final double ALIGN_SPEED_THRESHOLD = 0.1;
    public static final double ALIGN_SPEED_FAST = 1.2;
    public static final double ALIGN_SPEED_SLOW = 0.8;

    // MANIPUlATIR Subsystem
    public static final int SHOOTER_MOTOR_A_ID = 32;
    public static final int SHOOTER_MOTOR_B_ID = 31;
    public static final int INTAKE_MOTOR_ID = 33;
    public static final int SENSOR_PORT = 18;

    // ANGLE Calculator
    public static final double armLength = 0.62;
    public static final double armAngleOffset = Math.toRadians(117);
    public static final double deltaHeight = 2 - 0.36;
    public static final double launchVelocity = 15.5;
    public static final double dragCoefficient = 0.0; // Positive - TODO
    public static final double FEED_FORWARD_VOLTAGE = 0.3;

    // CLIMBER Subsystem
    public static final int LEFT_CLIMBER_MOTOR_ID = 8;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 9;

    // Auto Static Groups

    public static final double ALL_GROUPS_ROTATION = -90;

    public static final int[] A_GROUP_MEMBERS = { 1, 2, 3 };
    public static final double A_GROUP_X = 0;
    public static final double A_GROUP_Y = 0;

    public static final int[] B_GROUP_MEMBERS = { 4, 5, 6 };
    public static final double B_GROUP_X = 0;
    public static final double B_GROUP_Y = 0;

    public static final int[] C_GROUP_MEMBERS = { 7, 8, 9 };
    public static final double C_GROUP_X = 6.95;
    public static final double C_GROUP_Y = 5.50;

    public static final int[] D_GROUP_MEMBERS = { 10, 11, 12 };
    public static final double D_GROUP_X = 0;
    public static final double D_GROUP_Y = 0;

    // Vacuum Subsystem

    public static final int VAC_1_ID = 54;
    public static final int VAC_2_ID = 55;
    public static final int VAC_3_ID = 56;

}