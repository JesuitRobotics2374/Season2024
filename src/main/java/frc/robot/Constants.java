package frc.robot;

public class Constants {
    public static final String CAN_BUS_NAME_CANIVORE = "FastFD";
    public static final String CAN_BUS_NAME_ROBORIO = "rio";
    public static final String CAN_BUS_NAME_DRIVETRAIN = CAN_BUS_NAME_CANIVORE;

    // IO Controller definitions
    public static final int CONTROLLER_USB_PORT_DRIVER = 0; // Drivers Controller
    public static final int CONTROLLER_USB_PORT_OPERATOR = 1; // Ordanence operators controller

    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // CHASSIS Subsystem
    public static final String TEST_ROBORIO_SERIAL_NUMBER = "0316b2d6"; // serial number of Swervee roborio
    public static final int LEFT_ARM_MOTOR_ID = 0;

    // ARM Subsystem
    public static final int RIGHT_ARM_MOTOR_ID = 0;
    public static final int ARM_ENCODER_ID = 0;

    // MANIPULATOR Subsystem
    public static final int SHOOTER_MOTOR_A_ID = 0;
    public static final int SHOOTER_MOTOR_B_ID = 0;
    public static final int INTAKE_MOTOR_ID = 0;

}
