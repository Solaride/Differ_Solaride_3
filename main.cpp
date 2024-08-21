#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// Function to read the steering angle from the sensor
double getSteeringAngleFromSensor() {
    static double steeringAngle = 0.0; // Initial value in degrees
    steeringAngle += 1.0; // Increment the steering angle by 1 degree every cycle
    return steeringAngle;
}

// Function to calculate delta1
double calculateWheelSpeedOne(double steerAngle, double wheelbase, double kingpin) {
    double steerAngleRad = steerAngle * M_PI / 180.0;
    double tanDelta = std::tan(steerAngleRad);
    double numerator = wheelbase * tanDelta;
    double denominator = wheelbase - (0.5 * kingpin * tanDelta);
    double delta1 = std::atan(numerator / denominator);
    return delta1;
}

// Function to calculate delta1
double calculateWheelSpeedTwo(double steerAngle, double wheelbase, double kingpin) {
    // Calculate the tangent of delta
    double steerAngleRad = steerAngle * M_PI / 180.0;
    double tanDelta = std::tan(steerAngleRad);
    double numerator = wheelbase * tanDelta;
    double denominator = wheelbase + (0.5 * kingpin * tanDelta);
    double delta2 = std::atan(numerator / denominator);
    return delta2;
}

// Function to calculate the radii of the inside and outside wheels
void calculateRadii(double wheelbase, double track, double steerAngle, double& R1, double& R2, double& R3, double& R4, double kingpin) {
    // Calculate the radius of the turn (R)
    double frontInnerWheel = calculateWheelSpeedOne(steerAngle, wheelbase, kingpin);
    double frontOuterWheel = calculateWheelSpeedTwo(steerAngle, wheelbase, kingpin);
    steerAngle = steerAngle * M_PI / 180.0;


    // Calculate the radii for the inside and outside wheels
    R1 = wheelbase / std::tan(frontInnerWheel);
    R2 = wheelbase / std::tan(frontOuterWheel);
    R3 = wheelbase / std::tan(steerAngle) - track / 2;
    R4 = wheelbase / std::tan(steerAngle) + track / 2;
}

double calculateRcg(double R3, double dr, double lr) {
    // Calculate Rcg using the formula
    double Rcg = std::sqrt(std::pow(R3 + (dr / 2.0), 2) + std::pow(lr, 2));
    return Rcg;
}

// Function to calculate the RPM of the inside and outside wheels
void calculateWheelSpeeds(double vehicleSpeed, double& R1, double& R2, double& R3, double& R4, double& rpm_in, double& rpm_out, double& RPM1, double& RPM2, double wheelRadius, double track, double gravity) {
    double Rcg = calculateRcg(R3, track, gravity);

    double W1 = vehicleSpeed * R1 / Rcg * wheelRadius;
    double W2 = vehicleSpeed * R2 / Rcg * wheelRadius;
    double W3 = vehicleSpeed * R3 / Rcg * wheelRadius;
    double W4 = vehicleSpeed * R4 / Rcg * wheelRadius;

    rpm_in = W1 / (wheelRadius * 2 * M_PI) * 60;
    rpm_out = W2 / (wheelRadius * 2 * M_PI) * 60;
    RPM1 = W3 / (wheelRadius * 2 * M_PI) * 60;
    RPM2 = W4 / (wheelRadius * 2 * M_PI) * 60;
}

int main() {
    double wheelbase = 3.2; // Distance between front and rear axles (in meters)
    double track = 1.5;     // Distance between left and right wheels (in meters)
    double vehicleSpeed = 50; // Vehicle speed (in km/h)
    double wheelRadius = 0.279; // Wheel diameter (in meters)
    double gravity = 1.296;
    double kingpin = 1.348;
    double R1, R2, R3, R4; // Radii for the inside and outside wheels
    double rpm_in, rpm_out, RPM1, RPM2; // RPM for the inside and outside wheels

    // Continuously read the steering angle and calculate the wheel speeds
    while (true) {
        // Get the current steering angle from the sensor
        double steerAngle = getSteeringAngleFromSensor();

        // Calculate the radii for the inside and outside wheels
        calculateRadii(wheelbase, track, steerAngle, R1, R2, R3, R4, kingpin);

        // Calculate the RPM for the inside and outside wheels
        calculateWheelSpeeds(vehicleSpeed, R1, R2, R3, R4, rpm_in, rpm_out, RPM1, RPM2,  wheelRadius, track, gravity);

        // Output the results
        std::cout << "Steering angle: " << steerAngle << " degrees\n";
        std::cout << "Front inside wheel speed: " << rpm_in << " RPM\n";
        std::cout << "Front outside wheel speed: " << rpm_out << " RPM\n";
        std::cout << "Rear inside wheel speed: " << RPM1 << " RPM\n";
        std::cout << "Rear outside wheel speed: " << RPM2 << " RPM\n";

        // Wait for a short period before reading the sensor again
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    return 0;
}
