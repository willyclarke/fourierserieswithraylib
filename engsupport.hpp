/*******************************************************************************************
 *
 *   Engineering support
 *
 *   BSD-like license that allows static linking with closed source software
 *
 *   Copyright (c) 2023 Willy Clarke (willyclarke@gmail.com)
 *
 ********************************************************************************************/
#ifndef ENGSUPPORT_HPP
#define ENGSUPPORT_HPP

#include "raylib.h"
#include <iostream>

/**
 * es - engineering support namespace
 */
namespace es // aka engineering support
{
void TestHomogenousMatrix();
auto Test3dCalucations() -> void;
auto Test3dScreenCalculations() -> void;

/**
 * Identity matrix 4x4
 */
Matrix I();

//------------------------------------------------------------------------------
/**
 * Return matrix 4x4 for conversion from engineering space to screen space.
 * Screen space is a floating point representation with x,y,z = 0,0,0 beeing at
 * the middle of the window.
 */
Matrix InitTranslationInv(Matrix const &M, Vector4 const &vTranslation);

/**
 * Defintions: A point in 3D space has w set to 1.
 *             A vector in 3D space has w set to 0.
 *             This implies that adding Vectors gives a new Vector.
 *             And Adding a Vector to a Point gives a new Point.
 *             And there is no meaning in adding Points since w would not be 1.
 */
Vector4 Point(float X, float Y, float Z);

/**
 * Defintions: A point in 3D space has w set to 1.
 *             A vector in 3D space has w set to 0.
 *             This implies that adding Vectors gives a new Vector.
 *             And Adding a Vector to a Point gives a new Point.
 *             And there is no meaning in adding Points since w would not be 1.
 */
Vector4 Vector(float X, float Y, float Z);

//------------------------------------------------------------------------------
/**
 * Return matrix 4x4 for conversion from engineering space to screen space.
 * Screen space is a floating point representation with x,y,z = 0,0,0 beeing at
 * the middle of the window.
 */
Matrix InitScaling(Matrix const &M, Vector4 const &Scale,
                   bool Reflection = false);

/**
 * Return the result of multiplication of a Matrix and a Vector, dimension 4.
 */
Vector4 Mul(Matrix const &M, Vector4 const &V);

//------------------------------------------------------------------------------
float Mul(Vector4 const &V1, Vector4 const &V2);

//------------------------------------------------------------------------------
Matrix Add(Matrix const &M1, Matrix const &M2);

//------------------------------------------------------------------------------
bool Eq(Matrix const &M1, Matrix const &M2);

//------------------------------------------------------------------------------
Matrix Mul(Matrix const &M1, Matrix const &M2);

}; // namespace es

Matrix operator*(Matrix const &M1, Matrix const &M2);
Matrix operator+(Matrix const &M1, Matrix const &M2);
bool operator==(Matrix const &M1, Matrix const &M2);
bool operator!=(Matrix const &M1, Matrix const &M2);
Vector4 operator*(Matrix const &M, Vector4 const &V);
float operator*(Vector4 const &V1, Vector4 const &V2);
Vector4 operator+(Vector4 const &V1, Vector4 const &V2);
Vector4 operator-(Vector4 const &V1, Vector4 const &V2);

std::ostream &operator<<(std::ostream &stream, const Vector4 &T);
std::ostream &operator<<(std::ostream &stream, const Matrix &M);

#endif
