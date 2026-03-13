#include "structures.h"

vector2::vector2(float x, float y):x(x), y(y){};

vector2 vector2::operator+(const vector2& v){
    return vector2(x+v.x, y+v.y);
}

vector2 vector2::operator-(const vector2& v){
    return vector2(x-v.x, y-v.y);
}