/*******************************************************************************************
 *
 *   Engineering support
 *
 *   BSD-like license that allows static linking with closed source software
 *
 *   Copyright (c) 2023 Willy Clarke (willyclarke@gmail.com)
 *
 ********************************************************************************************/

#include "engsupport.hpp"
#include "raylib.h"
#include <iomanip>

/**
 * es - engineering support namespace
 */
namespace es // aka engineering support
{
/**
 * Identity matrix 4x4
 */
Matrix I() {
  Matrix M{};
  M.m0 = 1.f;
  M.m5 = 1.f;
  M.m10 = 1.f;
  M.m15 = 1.f;
  return M;
}

//------------------------------------------------------------------------------
/**
 * Return matrix 4x4 for conversion from engineering space to screen space.
 * Screen space is a floating point representation with x,y,z = 0,0,0 beeing at
 * the middle of the window.
 */
Matrix InitTranslationInv(Matrix const &M, Vector4 const &vTranslation) {
  Matrix Mat = M;
  // float m0, m4, m8, m12;  // Matrix first row (4 components)
  // float m1, m5, m9, m13;  // Matrix second row (4 components)
  // float m2, m6, m10, m14; // Matrix third row (4 components)
  // float m3, m7, m11, m15; // Matrix fourth row (4 components)
  Mat.m0 = 1.f * M.m0;
  Mat.m5 = 1.f * M.m5;
  Mat.m10 = 1.f * M.m10;
  Mat.m12 = -vTranslation.x;
  Mat.m13 = -vTranslation.y;
  Mat.m14 = -vTranslation.z;
  Mat.m15 = 1.f;

  return Mat;
}

/**
 * Defintions: A point in 3D space has w set to 1.
 *             A vector in 3D space has w set to 0.
 *             This implies that adding Vectors gives a new Vector.
 *             And Adding a Vector to a Point gives a new Point.
 *             And there is no meaning in adding Points since w would not be 1.
 */
Vector4 Point(float X, float Y, float Z) { return Vector4{X, Y, Z, 1.f}; }

/**
 * Defintions: A point in 3D space has w set to 1.
 *             A vector in 3D space has w set to 0.
 *             This implies that adding Vectors gives a new Vector.
 *             And Adding a Vector to a Point gives a new Point.
 *             And there is no meaning in adding Points since w would not be 1.
 */
Vector4 Vector(float X, float Y, float Z) { return Vector4{X, Y, Z, 0.f}; }

/**
 * Return matrix 4x4 for conversion from engineering space to screen space.
 * Screen space is a floating point representation with x,y,z = 0,0,0 beeing at
 * the middle of the window.
 */
Matrix InitScaling(Matrix const &M, Vector4 const &Scale, bool Reflection) {
  Matrix Mat = M;
  auto const Sign = Reflection ? -1.f : 1.f;
  Mat.m0 = Sign * Scale.x;
  Mat.m5 = Sign * Scale.y;
  Mat.m10 = Sign * Scale.z;
  Mat.m15 = 1.f;

  return Mat;
}

/**
 * Return the result of multiplication of a Matrix and a Vector, dimension 4.
 */
Vector4 Mul(Matrix const &M, Vector4 const &V) {
  Vector4 Result{};
  Result.x = M.m0 * V.x + M.m4 * V.y + M.m8 * V.z + M.m12 * V.w;
  Result.y = M.m1 * V.x + M.m5 * V.y + M.m9 * V.z + M.m13 * V.w;
  Result.z = M.m2 * V.x + M.m6 * V.y + M.m10 * V.z + M.m14 * V.w;
  Result.w = M.m3 * V.x + M.m7 * V.y + M.m11 * V.z + M.m15 * V.w;
  return Result;
}

/**
 */
Vector4 Add(Vector4 const &V1, Vector4 const &V2) {
  return Vector4{V1.x + V2.x, V1.y + V2.y, V1.z + V2.z, V1.w + V2.w};
}

/**
 * Dot product.
 */
float Mul(Vector4 const &V1, Vector4 const &V2) {
  return V1.x * V2.x + V1.y * V2.y + V1.z * V2.z + V1.w * V2.w;
}

/**
 */
Vector4 Sub(Vector4 const &V1, Vector4 const &V2) {
  return Vector4{V1.x - V2.x, V1.y - V2.y, V1.z - V2.z, V1.w - V2.w};
}

/**
 */
Matrix Add(Matrix const &M1, Matrix const &M2) {
  Matrix Result{};
  Result.m0 = M1.m0 + M2.m0;
  Result.m1 = M1.m1 + M1.m1;
  Result.m2 = M1.m2 + M2.m2;
  Result.m3 = M1.m3 + M2.m3;
  Result.m4 = M1.m4 + M2.m4;
  Result.m5 = M1.m5 + M2.m5;
  Result.m6 = M1.m6 + M2.m6;
  Result.m7 = M1.m7 + M2.m7;
  Result.m8 = M1.m8 + M2.m8;
  Result.m9 = M1.m9 + M2.m9;
  Result.m10 = M1.m10 + M2.m10;
  Result.m11 = M1.m11 + M2.m11;
  Result.m12 = M1.m12 + M2.m12;
  Result.m13 = M1.m13 + M2.m13;
  Result.m14 = M1.m14 + M2.m14;
  Result.m15 = M1.m15 + M2.m15;
  return Result;
}

/**
 */
bool Eq(Matrix const &M1, Matrix const &M2) {
  auto const Result = M1.m0 == M2.m0 &&   //!<
                      M1.m1 == M2.m1 &&   //!<
                      M1.m2 == M2.m2 &&   //!<
                      M1.m3 == M2.m3 &&   //!<
                      M1.m4 == M2.m4 &&   //!<
                      M1.m5 == M2.m5 &&   //!<
                      M1.m6 == M2.m6 &&   //!<
                      M1.m7 == M2.m7 &&   //!<
                      M1.m8 == M2.m8 &&   //!<
                      M1.m9 == M2.m9 &&   //!<
                      M1.m10 == M2.m10 && //!<
                      M1.m11 == M2.m11 && //!<
                      M1.m12 == M2.m12 && //!<
                      M1.m13 == M2.m13 && //!<
                      M1.m14 == M2.m14 && //!<
                      M1.m15 == M2.m15    //!<
      ;
  return Result;
}

/**
 */
Matrix Mul(Matrix const &M1, Matrix const &M2) {
  Matrix Result{};
  Result.m0 = Mul(Vector4{M1.m0, M1.m4, M1.m8, M1.m12},
                  Vector4{M2.m0, M2.m1, M2.m2, M2.m3});
  Result.m1 = Mul(Vector4{M1.m1, M1.m5, M1.m9, M1.m13},
                  Vector4{M2.m0, M2.m1, M2.m2, M2.m3});
  Result.m2 = Mul(Vector4{M1.m2, M1.m6, M1.m10, M1.m14},
                  Vector4{M2.m0, M2.m1, M2.m2, M2.m3});
  Result.m3 = Mul(Vector4{M1.m3, M1.m7, M1.m11, M1.m15},
                  Vector4{M2.m0, M2.m1, M2.m2, M2.m3});

  Result.m4 = Mul(Vector4{M1.m0, M1.m4, M1.m8, M1.m12},
                  Vector4{M2.m4, M2.m5, M2.m6, M2.m7});
  Result.m5 = Mul(Vector4{M1.m1, M1.m5, M1.m9, M1.m13},
                  Vector4{M2.m4, M2.m5, M2.m6, M2.m7});
  Result.m6 = Mul(Vector4{M1.m2, M1.m6, M1.m10, M1.m14},
                  Vector4{M2.m4, M2.m5, M2.m6, M2.m7});
  Result.m7 = Mul(Vector4{M1.m3, M1.m7, M1.m11, M1.m15},
                  Vector4{M2.m4, M2.m5, M2.m6, M2.m7});

  Result.m8 = Mul(Vector4{M1.m0, M1.m4, M1.m8, M1.m12},
                  Vector4{M2.m8, M2.m9, M2.m10, M2.m11});
  Result.m9 = Mul(Vector4{M1.m1, M1.m5, M1.m9, M1.m13},
                  Vector4{M2.m8, M2.m9, M2.m10, M2.m11});
  Result.m10 = Mul(Vector4{M1.m2, M1.m6, M1.m10, M1.m14},
                   Vector4{M2.m8, M2.m9, M2.m10, M2.m11});
  Result.m11 = Mul(Vector4{M1.m3, M1.m7, M1.m11, M1.m15},
                   Vector4{M2.m8, M2.m9, M2.m10, M2.m11});

  Result.m12 = Mul(Vector4{M1.m0, M1.m4, M1.m8, M1.m12},
                   Vector4{M2.m12, M2.m13, M2.m14, M2.m15});
  Result.m13 = Mul(Vector4{M1.m1, M1.m5, M1.m9, M1.m13},
                   Vector4{M2.m12, M2.m13, M2.m14, M2.m15});
  Result.m14 = Mul(Vector4{M1.m2, M1.m6, M1.m10, M1.m14},
                   Vector4{M2.m12, M2.m13, M2.m14, M2.m15});
  Result.m15 = Mul(Vector4{M1.m3, M1.m7, M1.m11, M1.m15},
                   Vector4{M2.m12, M2.m13, M2.m14, M2.m15});
  return Result;
}

//------------------------------------------------------------------------------
void TestHomogenousMatrix() {
  // float m0, m4, m8, m12;  // Matrix first row (4 components)
  // float m1, m5, m9, m13;  // Matrix second row (4 components)
  // float m2, m6, m10, m14; // Matrix third row (4 components)
  // float m3, m7, m11, m15; // Matrix fourth row (4 components)
  auto Hes = es::I();
  constexpr float Flip = -1.f;

  // Make 1 engineering unit correspond to 100 pixels.
  constexpr float Eng2Pixel = 100;

  // Set translation:
  Hes.m12 = 1280 / 2.f;
  Hes.m13 = 1024 / 2.f;

  // Some engineering test points.
  auto const Pe1 = es::Point(0.f, 0.f, 0.f);
  auto const Pe2 = es::Point(1.f, 0.f, 0.f);
  auto const Pe3 = es::Point(-1.f, 0.f, 0.f);
  auto const Ps = Hes * Pe1;

  std::cout << Hes << std::endl;
  std::cout << Ps << std::endl;

  // Flip and scale to pixel value.
  Hes.m0 = Flip * Eng2Pixel;
  Hes.m5 = Flip * Eng2Pixel;
  Hes.m10 = Flip * Eng2Pixel;
  std::cout << "Eng point " << Pe1 << " moves to screen coord " << Hes * Pe1
            << std::endl;
  std::cout << "Eng point " << Pe2 << " moves to screen coord " << Hes * Pe2
            << std::endl;
  std::cout << "Eng point " << Pe3 << " moves to screen coord " << Hes * Pe3
            << std::endl;

  // Screen translation
  Matrix Hst{};
  Hst.m12 = 100.f;
  Hst.m13 = 100.f;
  auto H = Hes + Hst;
  std::cout << "Eng point " << Pe1 << " moves to screen coord " << H * Pe1
            << std::endl;
  std::cout << "Eng point " << Pe2 << " moves to screen coord " << H * Pe2
            << std::endl;
  std::cout << "Eng point " << Pe3 << " moves to screen coord " << H * Pe3
            << std::endl;
}

/**
 */
auto Test3dCalucations() -> void {

  Matrix M{};

  // Move from engineering space to screen space
  // i.e. The center of the screen will be at x=3,y=4
  M = es::InitTranslationInv({}, es::Point(3.f, 4.f, 0.f));
  {
    Vector4 const V = M * Vector4{0.f, 0.f, 0.f, 1.f};
    // Expected output is -3, -4
    if (V.x != -3.f || V.y != -4.f)
      std::cerr << "Failed to compute screen coordinates. Line:" << __LINE__
                << std::endl;
    //
  }
  {
    Vector4 const V = M * es::Point(3.f, 0.f, 0.f);
    // Expected output is 0, -4
    if (V.x != 0.f || V.y != -4.f) {
      std::cerr << "Failed to compute screen coordinates. Line:" << __LINE__
                << std::endl;
      std::cerr << M << std::endl;
      std::cerr << V << std::endl;
    }
  }

  M = es::InitScaling({}, es::Point(2.f, 3.f, 4.f));
  std::cout << M << std::endl;

  // NOTE: Scaling applies to vectors and points.
  Vector4 const Po = M * Vector4{-4.f, 6.f, 8.f, 1.f};
  if (Po.x != -8.f || Po.y != 18.f || Po.z != 32.f)
    std::cerr << "Failed to compute scaling. Line:" << __LINE__ << std::endl;

  Vector4 const Vector = M * Vector4{-4.f, 6.f, 8.f, 0.f};
  if (Vector.x != -8.f || Vector.y != 18.f || Vector.z != 32.f) {
    std::cerr << "Failed to compute scaling. Line:" << __LINE__ << std::endl;

    std::cout << Po << std::endl;
    std::cout << Vector << std::endl;
  }

  // ---
  // Test Point multiplication with matrix.
  // ---
  {
    Matrix M2{
        1.f, 2.f, 3.f, 4.f, //<!
        2.f, 4.f, 4.f, 2.f, //<!
        8.f, 6.f, 4.f, 1.f, //<!
        0.f, 0.f, 0.f, 1.f  //!<
    };

    Vector4 const P = es::Point(1.f, 2.f, 3.f);
    auto const Result = es::Mul(M2, P);
    if (Result.x != 18.f || Result.y != 24.f || Result.z != 33.f ||
        Result.w != 1.f) {

      std::cerr << "Failed to compute Matrix multiplicator, Line: " << __LINE__
                << std::endl;

      std::cout << M2 << std::endl;
      std::cout << Result << std::endl;
    }
  }

  // ---
  // Test Point multiplication with matrix using operators.
  // ---
  {
    Matrix M2{
        1.f, 2.f, 3.f, 4.f, //<!
        2.f, 4.f, 4.f, 2.f, //<!
        8.f, 6.f, 4.f, 1.f, //<!
        0.f, 0.f, 0.f, 1.f  //!<
    };

    Vector4 const P = es::Point(1.f, 2.f, 3.f);
    auto const Result = M2 * P;
    if (Result.x != 18.f || Result.y != 24.f || Result.z != 33.f ||
        Result.w != 1.f) {

      std::cerr << "Failed to compute Matrix multiplicator, Line: " << __LINE__
                << std::endl;

      std::cout << M2 << std::endl;
      std::cout << Result << std::endl;
    }
  }

  // ---
  // Test Matrix multiplication with Matrix
  // ---
  {
    Matrix A{
        1.f, 2.f, 3.f, 4.f, //!<
        5.f, 6.f, 7.f, 8.f, //!<
        9.f, 8.f, 7.f, 6.f, //!<
        5.f, 4.f, 3.f, 2.f  //!<
    };
    Matrix B{
        -2.f, 1.f, 2.f, 3.f,  //!<
        3.f,  2.f, 1.f, -1.f, //!<
        4.f,  3.f, 6.f, 5.f,  //!<
        1.f,  2.f, 7.f, 8.f   //!<
    };
    Matrix Expect{
        20.f, 22.f, 50.f,  48.f,  //!<
        44.f, 54.f, 114.f, 108.f, //!<
        40.f, 58.f, 110.f, 102.f, //!<
        16.f, 26.f, 46.f,  42.f   //!<
    };

    auto const M = A * B;
    if (M != Expect) {
      std::cerr << "Matrix multiplication failed. Line: " << __LINE__
                << std::endl;
      std::cerr << "Calculated Matrix:" << M << std::endl;
      std::cerr << "Expected Matrix:" << Expect << std::endl;
    }
  }
}

/**
 * Test changes from coordinate system to screen coordinates.
 */
auto Test3dScreenCalculations() -> void {
  // Given the engineering input
  auto const Pe = es::Point(0.f, 0.f, 0.f);

  // Compute the screen coordinates - aka Matrix Screen = Ms
  auto const Ms = es::InitTranslationInv({}, es::Point(0.f, 0.f, 0.f));
  auto const Ps = Ms * Pe;

  std::cout << "Ms : " << Ms << std::endl;
  std::cout << "Pe :" << Pe << std::endl;
  std::cout << "Ps :" << Ps << std::endl;

  // Compute the scaling into pixel space - aka Matrix Pixel Scale = Mps
  // That gives the Pixel Point Pp.
  auto const Mps = es::InitScaling({}, es::Point(100.f, 100.f, 100.f));
  auto const Pps = Mps * Ps;

  std::cout << "Mps: " << Mps << std::endl;
  std::cout << "Pps:" << Pps << std::endl;

  // Compute the translation onto the screen based on 0,0 beeing top left of
  // screen
  // Matrix Translation - aka Mpt
  auto const Mpt =
      es::InitTranslationInv({}, es::Point(-1280.f / 2.f, -1024.f / 2.f, 0.f));
  std::cout << "Mpt:" << Mpt << std::endl;

  {
    auto const Pp = Mpt * Pps;
    std::cout << "Pp :" << Pp << std::endl;
  }

  // ---
  // Combine all the matrixes into one by multiplication.
  // ---
  {
    auto const M = Mpt * Mps * Ms;
    auto const PixelPos = Mpt * Pps;
    std::cout << "Resulting Matrix:" << M << std::endl;
    std::cout << "Pixel xositi :" << PixelPos << std::endl;
    std::cout << "Pixel cositi :" << M * Pe << std::endl;
  }
}
}; // namespace es

Matrix operator*(Matrix const &M1, Matrix const &M2) { return es::Mul(M1, M2); }
Matrix operator+(Matrix const &M1, Matrix const &M2) { return es::Add(M1, M2); }
bool operator==(Matrix const &M1, Matrix const &M2) { return es::Eq(M1, M2); }
bool operator!=(Matrix const &M1, Matrix const &M2) { return !(M1 == M2); }
Vector4 operator*(Matrix const &M, Vector4 const &V) { return es::Mul(M, V); }
Vector4 operator+(Vector4 const &V1, Vector4 const &V2) {
  return es::Add(V1, V2);
}
float operator*(Vector4 const &V1, Vector4 const &V2) {
  return es::Mul(V1, V2);
}
Vector4 operator-(Vector4 const &V1, Vector4 const &V2) {
  return es::Sub(V1, V2);
}

/**
 */
std::ostream &operator<<(std::ostream &stream, const Vector4 &T) {
  // ---
  // NOTE: The width need to be big enough to hold a negative sign.
  // ---
  size_t const P{5};
  size_t const W{P + 5};
  stream << ((T.w != 0) ? "Point :" : "Vector:");
  stream << " " << std::fixed << std::setprecision(P) << std::setw(W) << T.x
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << T.y
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << T.z
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << T.w;
  return stream;
}

/**
 */
std::ostream &operator<<(std::ostream &stream, const Matrix &M) {
  // ---
  // NOTE: The width need to be big enough to hold a negative sign.
  // ---
  size_t const P{5};
  size_t const W{P + 5};
  stream << "Matrix\n";
  stream << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m0
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m4
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m8
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m12
         << "\n";
  stream << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m1
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m5
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m9
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m13
         << "\n";
  stream << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m2
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m6
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m10
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m14
         << "\n";
  stream << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m3
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m7
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m11
         << " " << std::fixed << std::setprecision(P) << std::setw(W) << M.m15
         << "\n";
  return stream;
}
