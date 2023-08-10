//
// Created by LEI XU on 5/13/19.
//

#include "Vector.hpp"
double Vector3f::len(){
    return sqrt(dotProduct((*this), (*this)));
}