#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <array>
#include <fstream> // for excel
#include <iomanip> // for excel

#define rad2deg 180 / M_PI
#define deg2rad M_PI / 180
// watch() is only to be used for debugging purpose during development
#define watch(x) std::cout << (#x) << ":\n " << (x) << std::endl

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main()
{
//Initialise platform defaults here
    double X, Y, Z, phi, theta, psi;
    double servo_arm = 18;
    double servo_leg = 140;
    double beta[6] = {
                        M_PI / 3,
                        -2 * M_PI / 3,
                        M_PI,
                        0,
                        5 * M_PI / 3,
                        2 * M_PI / 3}; //! Changes home calcs
    double height = 155;
    int FLAG = 0;

    // Platform points (top) Pi
    MatrixXd Platform_pos_zero{
        {-41.4, 38.3, 0},
        {-53.9, 16.7, 0},
        {-12.5, -55, 0},
        {12.5, -55, 0},
        {53.9, 16.7, 0},
        {41.4, 38.3, 0}};
    Platform_pos_zero.transposeInPlace();
    watch(Platform_pos_zero); // debug

    // Servo points (base) //! z changed to all zeros
    MatrixXd Servo_pos{
        {-46.5, 74, 0},
        {-90.5, 1.1, 0},
        {-43.5, -81, 0},
        {43.5, -81, 0},
        {90.5, 1.14, 0},
        {46.5, 74, 0}};
    Servo_pos.transposeInPlace();
    watch(Servo_pos); // debug

    // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
    MatrixXd R_PB{
        {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
        {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
        {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

    };

    // This is where we open the data.csv file to write updates into for the animation

    std::fstream file;
    file << std::fixed << std::setprecision(3); // set precision to 3 decimal places
    file.open("data.csv", std::ios::out);




// Initialising to home position
    while(FLAG == 0)
    {
       //Parameters (3 trans inputs and 3 rot inputs)
       X = Y = Z = phi = theta = psi = 0;

       // Height of the platform when servo arm is perpendicular to leg
       double h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
               - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
               - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
               - Platform_pos_zero(2,0);
                 watch(h_0);//debug

       // Platform points when zero rotation and translation
       Eigen::Matrix<double, 3, 1> t_home;
       t_home << 0, 0, h_0;
       Eigen::Matrix<double, 3, 1> t_input;
       t_input << X, Y, Z;
       Eigen::Matrix<double, 3, 1> T;
       T = t_home + t_input;
       watch(t_home);//debug
       watch(t_input);//debug
       watch(T);//debug
       watch(R_PB);//debug

       // Calculate platform's home position (New_pos)
       MatrixXd Rotated_platform = R_PB * Platform_pos_zero;
       MatrixXd New_pos = T.replicate<1, 6>().array() + Rotated_platform.array(); // q
       watch(Rotated_platform);//debug
       watch(New_pos);//debug

       // Calculate angle of the servo arm at home position
       // First finding the linear Leg length

       MatrixXd lin_leg_lengths = New_pos - Servo_pos;
       watch(lin_leg_lengths);//debug
       // The .colwise().norm() method calculates the Euclidean norm of each column,
       // which corresponds to the length of each leg vector
       MatrixXd virtual_leg_lengths = (lin_leg_lengths).colwise().norm();
       watch(virtual_leg_lengths);



       // Due to platform symmetry, we can only consider the leg with 0 beta
       double L_home = 2 * std::pow(servo_arm, 2);
       double M_home = 2 * servo_arm * (New_pos(0,0) - Servo_pos(0,0));
       double N_home = 2 * servo_arm * (h_0 + New_pos(2,3));

       double alpha_home = asin(L_home/sqrt(pow(M_home,2)+pow(N_home,2))) - atan(M_home/N_home);
       double alpha_home_deg = rad2deg*alpha_home;
       watch(L_home);//debug
       watch(M_home);//debug
       watch(N_home);//debug
       watch(alpha_home);//debug
       watch(alpha_home_deg);//debug

      // Calculating the servo arm/leg join positions


       Eigen::Matrix<double,3,6> Knee_pos;

                for (size_t j=0; j<6; j++)
                {
                    Knee_pos(0,j) = servo_arm*cos(alpha_home)*cos(beta[j]) + Servo_pos(0,j);
                    Knee_pos(1,j) = servo_arm*cos(alpha_home)*sin(beta[j]) + Servo_pos(1,j);
                    Knee_pos(2,j) = servo_arm*sin(alpha_home) + Servo_pos(2,j);
                }

                  watch(Knee_pos);


        FLAG = 1;
    };

// When platform is moving
    while(FLAG == 1)
    {


    // Parameters (3 translational inputs and 3 rotational inputs)
    double X = 0;
    double Y = 0;
    double Z = -10;
    double phi = 0;//-0.087266462599717;
    double theta = 0;//0.226892802759263;
    double psi = 0.2;//0.191986217719376;

    // Height of the platform when servo arm is perpendicular to leg
       double h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
               - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
               - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
               - Platform_pos_zero(2,0);
                 watch(h_0);//debug

       // Platform points when zero rotation and translation
       Eigen::Matrix<double, 3, 1> t_home;
       t_home << 0, 0, h_0;
       Eigen::Matrix<double, 3, 1> t_input;
       t_input << X, Y, Z;
       Eigen::Matrix<double, 3, 1> T;
       T = t_home + t_input;
       watch(t_home);//debug
       watch(t_input);//debug
       watch(T);//debug
       watch(R_PB);//debug
       watch(phi);//debug


       // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
        MatrixXd R_PB{
        {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
        {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
        {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

    };

       // Calculate platform's transformed position(New_pos)
       MatrixXd Rotated_platform = R_PB * Platform_pos_zero;
       MatrixXd New_pos = T.replicate<1, 6>().array() + Rotated_platform.array(); // q
       watch(Rotated_platform);//debug
       watch(New_pos);//debug

       // Calculate angle of the servo arm at NEW position
       // First find the linear Leg length

       MatrixXd lin_leg_lengths = New_pos - Servo_pos;
       watch(lin_leg_lengths);//debug
       // The .colwise().norm() method calculates the Euclidean norm of each column,
       // which corresponds to the length of each leg vector
       MatrixXd virtual_leg_lengths = (lin_leg_lengths).colwise().norm();
       watch(virtual_leg_lengths);

       // Calculate the servo angles for each leg

       Eigen::Matrix<double,1,6> L;
        for (size_t i=0;i<6;i++)
        {

            L(0,i) = std::pow(virtual_leg_lengths(0,i),2)
                     - ((std::pow(servo_leg,2)) - (std::pow(servo_arm,2)));

        }

        watch(L);//debug

        Eigen::Matrix<double,1,6> M;
        for (size_t i=0;i<6;i++)
        {
            M(0,i) = 2*servo_arm*(New_pos(2,i)-Servo_pos(2,i));

        }

        watch(M);//debug

        Eigen::Matrix<double,1,6> N;
        for (size_t i=0;i<6;i++)
        {
            double x_diff  = New_pos(0,i) - Servo_pos(0,i); // intermediate calc
            double y_diff  = New_pos(1,i) - Servo_pos(1,i);
            N(0,i) = 2*servo_arm*((cos(beta[i])*x_diff)+(sin(beta[i])*y_diff));
        }

        watch(N);//debug

        // Now we can calculate the servo angles
        Eigen::Matrix<double,1,6>alpha;
        Eigen::Matrix<double,1,6>servo_deg;

        for (size_t i = 0; i <6; i++)
        {

            alpha(0,i) = asin(L(0,i) / sqrt(std::pow(M(0,i), 2) + std::pow(N(0,i), 2))) - atan(N(0,i) / M(0,i));
            servo_deg(0,i) = rad2deg*(alpha[i]);


        }

        watch(alpha);
        watch(servo_deg);

        // Let's save it all in the CSV file we opened previously. This will update the simulation.
        New_pos.transposeInPlace();
        for (size_t i = 0; i<6; i++)
        {
            for (size_t j = 0; j<3; j++)

            {
                file << New_pos(i,j);
                file << ";";
            }
             file << "\n";

        }
     //Changed knee position
     Eigen::Matrix<double,3,6> Knee_pos_new;

                for (size_t j=0; j<6; j++)
                {
                    Knee_pos_new(0,j) = servo_arm*cos(alpha[j])*cos(beta[j]) + Servo_pos(0,j);
                    Knee_pos_new(1,j) = servo_arm*cos(alpha[j])*sin(beta[j]) + Servo_pos(1,j);
                    Knee_pos_new(2,j) = servo_arm*sin(alpha[j]) + Servo_pos(2,j);
                }

                  watch(Knee_pos_new);
                // Updating the new knee positions in the opened CSV file
                MatrixXd Knee_pos_csv = Knee_pos_new;
                  Knee_pos_csv.transposeInPlace();
                  for (size_t i = 0; i<6; i++)
                    {
                        for (size_t j = 0; j<3; j++)

                        {
                            file << Knee_pos_csv(i,j);
                            file << ";";
                        }
                        file << "\n";

                    }


        file.close();



       FLAG = 0;


    };





    return 0;
}
