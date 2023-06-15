#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <array>
#include <fstream> // for excel
#include <iomanip> // for excel
#include <QDebug>

#define rad2deg 180 / M_PI
#define deg2rad M_PI / 180
// Used only for debugging during development
#define watch(x) std::cout << (#x) << ":\n " << (x) << std::endl

using Eigen::MatrixXd;
using Eigen::VectorXd;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->Slider_X, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_X_valueChanged(int)));
    connect(ui->Slider_Y, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Y_valueChanged(int)));
    connect(ui->Slider_Z, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Z_valueChanged(int)));
    connect(ui->Slider_PHI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PHI_valueChanged(int)));
    connect(ui->Slider_THETA, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_THETA_valueChanged(int)));
    connect(ui->Slider_PSI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PSI_valueChanged(int)));

}

MainWindow::~MainWindow()
{
    delete ui;

}

void MainWindow::on_pushButton_3_clicked()
{

       ui ->Slider_X->setValue(0);
       ui ->Slider_Y->setValue(0);
       ui ->Slider_Z->setValue(0);

       ui ->Slider_PHI->setValue(0);
       ui ->Slider_THETA->setValue(0);
       ui ->Slider_PSI->setValue(0);


    //Platform geometric parameters
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

        // Platform coordinates (top)
        MatrixXd Platform_pos_zero{
            {-41.4, 38.3, 0},
            {-53.9, 16.7, 0},
            {-12.5, -55, 0},
            {12.5, -55, 0},
            {53.9, 16.7, 0},
            {41.4, 38.3, 0}};
        Platform_pos_zero.transposeInPlace();


        // Servo coordingates (base)
        MatrixXd Servo_pos{
            {-46.5, 74, 0},
            {-90.5, 1.1, 0},
            {-43.5, -81, 0},
            {43.5, -81, 0},
            {90.5, 1.14, 0},
            {46.5, 74, 0}};
        Servo_pos.transposeInPlace();


        // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
        MatrixXd R_PB{
            {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
            {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
            {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

        };




    // Initialising to home position
        while(FLAG == 0)
        {
           //Parameters (3 translational inputs and 3 rotational inputs)
           X = Y = Z = phi = theta = psi = 0;

           // Height of the platform when servo arm is perpendicular to leg
           double h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
                   - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
                   - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
                   - Platform_pos_zero(2,0);


           // Platform points when zero rotation and translation
           Eigen::Matrix<double, 3, 1> t_home;
           t_home << 0, 0, h_0;
           Eigen::Matrix<double, 3, 1> t_input;
           t_input << X, Y, Z;
           Eigen::Matrix<double, 3, 1> T;
           T = t_home + t_input;


           // Calculate platform's home position (New_pos)
           MatrixXd Rotated_platform = R_PB * Platform_pos_zero;
           MatrixXd New_pos = T.replicate<1, 6>().array() + Rotated_platform.array(); // q


           // Calculate angle of the servo arm at home position
           // First calculating the linear Leg length

           MatrixXd lin_leg_lengths = New_pos - Servo_pos;

           // The .colwise().norm() method calculates the Euclidean norm of each column,
           // which corresponds to the length of each leg vector
           MatrixXd virtual_leg_lengths = (lin_leg_lengths).colwise().norm();





           // Due to platform symmetry, we can only consider the leg with 0 beta //!Are we symmetric?
           double L_home = 2 * std::pow(servo_arm, 2);
           double M_home = 2 * servo_arm * (New_pos(0,0) - Servo_pos(0,0));
           double N_home = 2 * servo_arm * (h_0 + New_pos(2,3));

           double alpha_home = asin(L_home/sqrt(pow(M_home,2)+pow(N_home,2))) - atan(M_home/N_home);
           double alpha_home_deg = rad2deg*alpha_home;

          // Let's try to workout the servo arm/leg join positions

           // For writing simulation outputs to CSV file

           std::fstream file;
           file << std::fixed << std::setprecision(3); // set precision to 3 decimal places
           file.open("data.csv", std::ios::out);



           // Let's save it all in a CSV
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
        //Update knee position
        Eigen::Matrix<double,3,6> Knee_pos_home;

                   for (size_t j=0; j<6; j++)
                   {
                       Knee_pos_home(0,j) = servo_arm*cos(alpha_home)*cos(beta[j]) + Servo_pos(0,j);
                       Knee_pos_home(1,j) = servo_arm*cos(alpha_home)*sin(beta[j]) + Servo_pos(1,j);
                       Knee_pos_home(2,j) = servo_arm*sin(alpha_home) + Servo_pos(2,j);
                   }


                   // Updating CSV file with new knee position information
                   MatrixXd Knee_pos_csv = Knee_pos_home;
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



            FLAG = 1;
        };
}
void MainWindow::on_pushButton_clicked()
{
    double servo_arm = 18;
    double servo_leg = 140;
    double beta[6] = {
                        M_PI / 3,
                        -2 * M_PI / 3,
                        M_PI,
                        0,
                        5 * M_PI / 3,
                        2 * M_PI / 3};
    double height = 155;


    // Platform coordinates (top)
    MatrixXd Platform_pos_zero{
        {-41.4, 38.3, 0},
        {-53.9, 16.7, 0},
        {-12.5, -55, 0},
        {12.5, -55, 0},
        {53.9, 16.7, 0},
        {41.4, 38.3, 0}};
    Platform_pos_zero.transposeInPlace();


    // SServo coordingates (base)
    MatrixXd Servo_pos{
        {-46.5, 74, 0},
        {-90.5, 1.1, 0},
        {-43.5, -81, 0},
        {43.5, -81, 0},
        {90.5, 1.14, 0},
        {46.5, 74, 0}};
    Servo_pos.transposeInPlace();


    // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
    MatrixXd R_PB{
        {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
        {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
        {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

    };
    double FLAG = 1;

    // When platform is moving
        while(FLAG == 1)
        {

           double h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
                   - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
                   - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
                   - Platform_pos_zero(2,0);
                     //watch(h_0);//debug

           // Platform points when zero rotation and translation
           Eigen::Matrix<double, 3, 1> t_home;
           t_home << 0, 0, h_0;
           Eigen::Matrix<double, 3, 1> t_input;
           t_input << X, Y, Z;
           qDebug() << "X here is :" << X;
           Eigen::Matrix<double, 3, 1> T;
           T = t_home + t_input;



           // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
        MatrixXd R_PB{
            {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
            {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
            {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

        };

           // Calculate platform's transformed position(New_pos)
           MatrixXd Rotated_platform = R_PB * Platform_pos_zero;
           MatrixXd New_pos = T.replicate<1, 6>().array() + Rotated_platform.array(); // q


           // Calculate angle of the servo arm at NEW position
           // First find the linear Leg length

           MatrixXd lin_leg_lengths = New_pos - Servo_pos;

           // The .colwise().norm() method calculates the Euclidean norm of each column,
           // which corresponds to the length of each leg vector
           MatrixXd virtual_leg_lengths = (lin_leg_lengths).colwise().norm();
           watch(virtual_leg_lengths);

           // Calculating the servo angles for each leg

           Eigen::Matrix<double,1,6> L;
            for (size_t i=0;i<6;i++)
            {

                L(0,i) = std::pow(virtual_leg_lengths(0,i),2)
                         - ((std::pow(servo_leg,2)) - (std::pow(servo_arm,2)));

            }



            Eigen::Matrix<double,1,6> M;
            for (size_t i=0;i<6;i++)
            {
                M(0,i) = 2*servo_arm*(New_pos(2,i)-Servo_pos(2,i));

            }



            Eigen::Matrix<double,1,6> N;
            for (size_t i=0;i<6;i++)
            {
                double x_diff  = New_pos(0,i) - Servo_pos(0,i); // intermediate calc
                double y_diff  = New_pos(1,i) - Servo_pos(1,i);
                N(0,i) = 2*servo_arm*((cos(beta[i])*x_diff)+(sin(beta[i])*y_diff));
            }



            // Now we can calculate the servo angles
            Eigen::Matrix<double,1,6>alpha;
            Eigen::Matrix<double,1,6>servo_deg;

            for (size_t i = 0; i <6; i++)
            {

                alpha(0,i) = asin(L(0,i) / sqrt(std::pow(M(0,i), 2) + std::pow(N(0,i), 2))) - atan(N(0,i) / M(0,i));
                servo_deg(0,i) = rad2deg*(alpha[i]);


            }


            // Updating CSV file

            std::fstream file;
            file << std::fixed << std::setprecision(3); // set precision to 3 decimal places
            file.open("data.csv", std::ios::out);



            // Let's save it all in a CSV
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


                    // For CSV
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




}

void MainWindow::on_pushButton_2_clicked()
{
    QApplication::quit();
}


void MainWindow::on_Slider_X_valueChanged(int value)
{
    double X =  1.0*value;//(ui->Slider_X->value());
    qDebug() << "X value changed to" << X;
    setX(X);
    on_pushButton_clicked();

}

void MainWindow::on_Slider_Y_valueChanged(int value)
{
    double Y = 1.0*value;//(ui ->Slider_Y ->value());
    setY(Y);
    on_pushButton_clicked();
}

void MainWindow::on_Slider_Z_valueChanged(int value)
{
    double Z = 1.0*value;//(ui ->Slider_Z -> value());
    setZ(Z);
    on_pushButton_clicked();
}

void MainWindow::on_Slider_PHI_valueChanged(int value)
{
    double phi = deg2rad*value;//(ui->Slider_PHI ->value()) ;
    setPHI(phi);
    on_pushButton_clicked();
}

void MainWindow::on_Slider_THETA_valueChanged(int value)
{
    double theta = deg2rad*value;//(ui ->Slider_THETA ->value());
    setTHETA(theta);
    on_pushButton_clicked();
}

void MainWindow::on_Slider_PSI_valueChanged(int value)
{
    double psi = deg2rad*value;//(ui ->Slider_PSI -> value());
    setPSI(psi);
    on_pushButton_clicked();
}

void MainWindow::setX(double value)
{
    X = value;
}
void MainWindow::setY(double value)
{
    Y = value;
}

void MainWindow::setZ(double value)
{
    Z = value;
}
void MainWindow::setPHI(double value)
{
    phi = value;
}
void MainWindow::setTHETA(double value)
{
    theta = value;
}
void MainWindow::setPSI(double value)
{
    psi = value;
}
