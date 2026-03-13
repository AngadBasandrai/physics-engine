#pragma once

struct vector2{
    float x;
    float y;

    vector2(float x, float y);

    vector2 operator+(const vector2& v);
    vector2 operator-(const vector2& v);
};