#include "main.h"
#include "lemlib/api.hpp"
//adds the controller 
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//adds motors with the left using ports 1 and 10 and the right using ports 2 and 9
pros::MotorGroup leftMotors({1, 10}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({2, 9}, pros::MotorGearset::green);
//adds the inertial measuring unit for port 21
pros::Imu imu(21);
//adds the ability to use LemLib drivetrain functions
lemlib::Drivetrain drivetrain(&leftMotors,
								&rightMotors,
								10,
								lemlib::Omniwheel::NEW_4,
								360,
								2);
//allows the ability to use lemLib controller functions
lemlib::ControllerSettings linearController(10,
											0,
											3,
											3,
											1,
											100,
											3,
											500,
											20);
// angular motion controller
lemlib::ControllerSettings angularController(2,
												0,
												10,
												3,
												1,
												100,
												3,
												500,
												0);
// sensors for odometry
lemlib::OdomSensors sensors(nullptr,
							nullptr,
							nullptr,
							nullptr,
							&imu);
lemlib::ExpoDriveCurve throttleCurve(3,
										10,
										1.019);
lemlib::ExpoDriveCurve steerCurve(3,
									10,
									1.019);
//Adds everything to the single chassis to use the lemLib chassis functions
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
//begins the code
void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();
	pros::Task screenTask([&](){
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        } });
}
void disabled() {}
//uses this when a competition begins
void competition_initialize() {}
ASSET(example_txt);
//runs this for the autonomous using the lemLub functions
void autonomous()
{
	chassis.moveToPose(20, 15, 90, 4000);
	chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
	chassis.waitUntil(10);
	chassis.cancelMotion();
	chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
	chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
	chassis.follow(example_txt, 15, 4000, false);
	chassis.waitUntil(10);
	pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
	chassis.waitUntilDone();
	pros::lcd::print(4, "pure pursuit finished!");
}
//runs when when the user controller is being used
void opcontrol()
{
	while (true)
	{
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(leftY, rightX);
		pros::delay(10);
	}
}