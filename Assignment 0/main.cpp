#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>



Eigen::Vector2f rotate(Eigen::Vector2f p, float alpha){
    Eigen::Matrix2f mat;
    mat << std::cos(alpha), -std::sin(alpha), std::sin(alpha), std::cos(alpha);
/*
    float l = p.norm();
    float beta = std::acos(p.x() / l);
    float x2 = l * std::cos(alpha + beta);
    float y2 = l * std::sin(alpha + beta);*/
    return mat * p;
}


int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f w(1.0f, 0.0f, 0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    // vector dot product
    std::cout << "Example of dot product" <<std::endl;
    std::cout << v.dot(w) << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << j << std::endl;
    // matrix add i + j
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << i * 2.0f << std::endl;
    // matrix multiply i * j
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << i * v << std::endl;


    Eigen::Vector2f P(2.0f, 1.0f);
    P = rotate(P, 45.0 / 180.0 * acos(-1));
    P = P + Eigen::Vector2f(1.0f, 2.0f);
    std::cout << P << std::endl;

    return 0;
}