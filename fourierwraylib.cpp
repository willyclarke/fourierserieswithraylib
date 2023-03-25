/*******************************************************************************************
 *
 *   raylib [core] example - Basic window (adapted for HTML5 platform)
 *
 *   This example is prepared to compile for PLATFORM_WEB, PLATFORM_DESKTOP and
 *PLATFORM_RPI As you will notice, code structure is slightly diferent to the
 *other examples... To compile it for PLATFORM_WEB just uncomment #define
 *PLATFORM_WEB at beginning
 *
 *   This example has been created using raylib 1.3 (www.raylib.com)
 *   raylib is licensed under an unmodified zlib/libpng license (View raylib.h
 *for details)
 *
 *   Copyright (c) 2015 Ramon Santamaria (@raysan5)
 *
 ********************************************************************************************/

#include "raylib.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
//------------------------------------------------------------------------------
struct pixel_pos {
  int X{};
  int Y{};
};

//------------------------------------------------------------------------------
//
// Convert from Engineering value to pixel position.
// @ X - Horisontal position
// @ Y - Vertical position
// @ x2P - x-axis to pixel conversion factor, from eng. unit to Pixels,(m, s)
// @ y2P - y-axis to pixel conversion factor, from eng. unit to Pixels, (i.e. m)
// @ Height, Width - Canvas dimension
pixel_pos Conv2Pix(float X, float Y, //!<
                   float X2P,        //!<
                   float Y2P) {
  pixel_pos Result{};
  Result.X = int(X * X2P) + (GetScreenWidth() >> 1);
  Result.Y = int(-Y * Y2P) + (GetScreenHeight() >> 1);
  return Result;
};

//------------------------------------------------------------------------------
struct square_wave_elem {
  float Amplitude{};
  float Yn{};
  float Xn{};
  float Theta{};
};

//------------------------------------------------------------------------------
struct data {
  int screenWidth = 1280;
  int screenHeight = 768;
  int Key{};
  int KeyPrv{};
  bool StopUpdate{};
  float X{};
  float Y{};
  float Time{};
  float Fx{};         //!< Fourier calculated series value.
  int n{1};           //!< Fourier series number of terms.
  float m2Pixel{100}; //!< Meter 2 pixel conversion - multiplicator.
  float s2Pixel{100}; //!< Seconds 2 pixel conversion - multiplicator.
  float dt{};
  float t{};
  std::vector<pixel_pos> vPixelPos{};
  std::vector<square_wave_elem> vSquareWaveElems{};
  std::vector<pixel_pos> vGridLines{};
};

/**
 * es - enginnering support namespace
 */
namespace es // aka enginnering support
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
Matrix InitTranslationInv(Matrix const &M, float Ex, float Ey, float Ez) {
  Matrix Mat = M;
  // float m0, m4, m8, m12;  // Matrix first row (4 components)
  // float m1, m5, m9, m13;  // Matrix second row (4 components)
  // float m2, m6, m10, m14; // Matrix third row (4 components)
  // float m3, m7, m11, m15; // Matrix fourth row (4 components)
  Mat.m0 = 1.f;
  Mat.m5 = 1.f;
  Mat.m10 = 1.f;
  Mat.m12 = -Ex;
  Mat.m13 = -Ey;
  Mat.m14 = -Ez;
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

//------------------------------------------------------------------------------
/**
 * Return matrix 4x4 for conversion from engineering space to screen space.
 * Screen space is a floating point representation with x,y,z = 0,0,0 beeing at
 * the middle of the window.
 */
Matrix InitScaling(Matrix const &M, float Sx, float Sy, float Sz) {
  Matrix Mat = M;
  // float m0, m4, m8, m12;  // Matrix first row (4 components)
  // float m1, m5, m9, m13;  // Matrix second row (4 components)
  // float m2, m6, m10, m14; // Matrix third row (4 components)
  // float m3, m7, m11, m15; // Matrix fourth row (4 components)
  Mat.m0 = Sx;
  Mat.m5 = Sy;
  Mat.m10 = Sz;
  Mat.m15 = 1.f;

  return Mat;
}

/**
 * Return the result of multiplication of a Matrix and a Vector, dimension 4.
 */
Vector4 Mul(Matrix const &M, Vector4 const &V) {
  Vector4 Result{};
  // float x;                // Vector x component
  // float y;                // Vector y component
  // float z;                // Vector z component
  // float w;                // Vector w component
  Result.x = M.m0 * V.x + M.m4 * V.y + M.m8 * V.z + M.m12 * V.w;
  Result.y = M.m1 * V.x + M.m5 * V.y + M.m9 * V.z + M.m13 * V.w;
  Result.z = M.m2 * V.x + M.m6 * V.y + M.m10 * V.z + M.m14 * V.w;
  Result.w = M.m3 * V.x + M.m7 * V.y + M.m11 * V.z + M.m15 * V.w;
  return Result;
}

//------------------------------------------------------------------------------
float Mul(Vector4 const &V1, Vector4 const &V2) {
  return V1.x * V2.x + V1.y * V2.y + V1.z * V2.z + V1.w * V2.w;
}

Matrix Mul(Matrix const &M1, Matrix const &M2) {
  Matrix Result{};
  // float m0, m4, m8, m12;  // Matrix first row (4 components)
  // float m1, m5, m9, m13;  // Matrix second row (4 components)
  // float m2, m6, m10, m14; // Matrix third row (4 components)
  // float m3, m7, m11, m15; // Matrix fourth row (4 components)
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

}; // namespace es

Matrix operator*(Matrix const &M1, Matrix const &M2) { return es::Mul(M1, M2); }
bool operator==(Matrix const &M1, Matrix const &M2) {
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
bool operator!=(Matrix const &M1, Matrix const &M2) {
  auto const Result = !(M1 == M2);
  return Result;
}
Vector4 operator*(Matrix const &M, Vector4 const &V) { return es::Mul(M, V); }
float operator*(Vector4 const &V1, Vector4 const &V2) {
  return es::Mul(V1, V2);
}
//
// ---
// NOTE: Stream operator
// ---
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

//------------------------------------------------------------------------------
std::ostream &operator<<(std::ostream &stream, const Matrix &M) {
  // ---
  // NOTE: The width need to be big enough to hold a negative sign.
  // ---
  // float m0, m4, m8, m12;  // Matrix first row (4 components)
  // float m1, m5, m9, m13;  // Matrix second row (4 components)
  // float m2, m6, m10, m14; // Matrix third row (4 components)
  // float m3, m7, m11, m15; // Matrix fourth row (4 components)
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
//------------------------------------------------------------------------------
// Compute fractals
//------------------------------------------------------------------------------

/**
 * Compute Zn^2 + C
 */
Vector2 ComputeNext(Vector2 const &Current, Vector2 const &Constant) {
  // Z^2
  float const Zr = Current.x * Current.x - Current.y * Current.y;
  float const Zi = 2.f * Current.x * Current.y;

  // Add constant
  Vector2 const Result{Zr + Constant.x, Zi + Constant.y};
  return Result;
}

//----------------------------------------------------------------------------------
// Module Functions Declaration
//

//----------------------------------------------------------------------------------
void UpdateDrawFrame(data *Data); // Update and Draw one frame

//------------------------------------------------------------------------------
// @ desc Compute the amplitude of a single term of the Fourier based square
// @ wave.
// @ input term - the n'th term of the series.
// @ return - the amplitude of the term.
float FuncFourierSquareWaveAmplitude(int nthterm) {
  constexpr float FourOverPI = 4.f / M_PI;
  return FourOverPI / (2.f * float(nthterm) - 1.f);
}

//------------------------------------------------------------------------------
// @desc - Calculate each element of the Fourier square wave
// @NOTE: This function has side effect - it will update each element with the
//        current Theta angle and the value of each term.
// @return - Sum of n elements of the Fourier square wave.
float FuncFourierSquareWave(std::vector<square_wave_elem> &vSquareWaveElems,
                            float Omegat) {
  float Sum{};
  float n{1};
  for (auto &E : vSquareWaveElems) {
    auto const Angle{(2.f * n - 1.f) * Omegat};
    E.Yn = E.Amplitude * std::sin((Angle));
    E.Xn = E.Amplitude * std::cos(Angle);
    E.Theta = Omegat;
    Sum += E.Yn;
    n += 1.f;
  }
  return Sum;
}

//------------------------------------------------------------------------------
// @ desc Compute square wave based on n terms fourier series.
// @ input float t - time
// @ input n - number of terms
// @ output f(t) - at time t
// @ link :
// https://www.math.kit.edu/iana3/lehre/fourierana2014w/media/fstable141127.pdf
// @ function number 6
float FuncFourierSquareWave(float Theta, int N) {
  float Sum{};
  for (int n = 0; n < N; ++n) {
    Sum += std::sin((2.0 * float(n) - 1.0) * Theta) / (2.0 * float(n) - 1.0);
  }

  constexpr float FourOverPI = 4.0 / M_PI;
  float const Result = FourOverPI * Sum;

  return Result;
}

//------------------------------------------------------------------------------
// @ desc Compute n elements of the square wave amplitue
// @      Side effect: Set number of terms into data struct.
//
void InitFourierSquareWave(data &Data, int n) {

  Data.n = std::max(1, n);
  Data.vSquareWaveElems.clear();

  for (int Idx = 1; Idx <= Data.n; ++Idx) {
    square_wave_elem Element{};
    Element.Amplitude = 4.0 / M_PI / (2.0 * float(Idx) - 1);
    Element.Yn = 0.0;
    Element.Theta = 0.0;
    Data.vSquareWaveElems.push_back(Element);
  }
}

/*
 * Convert from Engineering space to Grid space.
 * This change of basis allows input of engineering X, Y values
 * and the transformation to the coordinates for the Grid.
 */
auto ComputeGridPosition(Vector2 const &E, Vector2 const &I) -> Vector2 {
  Matrix const InvM{1.f, 0.f, 0.f, -E.x, 0.f, 1.f, 0.f, -E.y,
                    0.f, 0.f, 1.f, 0.f,  0.f, 0.f, 0.f, 1.f};
  auto const V = Vector4{I.x, I.y, 0.f, 1.f};

  /* Result = InvM * V */
  Vector2 const Result{InvM.m0 * V.x + InvM.m12 * V.w,
                       InvM.m5 * V.y + InvM.m13 * V.w};
  return Result;
}

/**
 * Test function for ComputeGridPosition.
 */
auto TestComputeGridPosition() -> void {

  std::cout << __FUNCTION__ << "-> Run" << std::endl;

  {
    auto Result = ComputeGridPosition(Vector2{3.f, 1.f}, Vector2{0.f, 0.f});
    auto const Expect = Vector2{-3.f, -1.f};

    if (Result.x == Expect.x && Result.y == Expect.y)
      std::cout << "SUCCESS!" << std::endl;
    else
      std::cerr << "FAILURE" << std::endl;
  }
  {
    auto Result = ComputeGridPosition(Vector2{0.f, 0.f}, Vector2{1.f, 1.f});
    auto const Expect = Vector2{1.f, 1.f};
    if (Result.x == Expect.x && Result.y == Expect.y)
      std::cout << "SUCCESS!" << std::endl;
    else
      std::cerr << "FAILURE" << std::endl;
  }
}

//------------------------------------------------------------------------------
auto Test3dCalucations() -> void {

  Matrix M{};

  // Move from enginnering space to screen space
  // i.e. The center of the screen will be at x=3,y=4
  M = es::InitTranslationInv({}, 3.f, 4.f, 0.f);
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

  M = es::InitScaling({}, 2.f, 3.f, 4.f);
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
  // Given the enginnering input
  auto const EngPoint = es::Point(0.f, 0.f, 0.f);

  // Compute the screen coordinates

  auto const MatEng2Screen = es::InitTranslationInv({}, 0.f, 0.f, 0.f);
  auto const ScreenPoint = MatEng2Screen * EngPoint;

  std::cout << "Engineering2Screen Matrix: " << MatEng2Screen << std::endl;
  std::cout << "Eng coord    :" << EngPoint << std::endl;
  std::cout << "Screen coord :" << ScreenPoint << std::endl;

  auto const MatScreen2Pixel = es::InitScaling({}, 100.f, 100.f, 100.f);
  auto const PixelPoint = MatScreen2Pixel * ScreenPoint;
  std::cout << "Screen2Pixel Matrix: " << MatScreen2Pixel << std::endl;
  std::cout << "Pixel point  :" << PixelPoint << std::endl;

  auto const MatPixelTranslation =
      es::InitTranslationInv({}, -1280.f / 2.f, -1024.f / 2.f, 0.f);

  {
    auto const PixelPos = MatPixelTranslation * PixelPoint;
    std::cout << "Pixel positi :" << PixelPos << std::endl;
  }

  // ---
  // Combine all the matrixes into one by multiplication.
  // ---
  {
    auto const M = MatEng2Screen * MatScreen2Pixel * MatPixelTranslation;
    auto const PixelPos = MatPixelTranslation * PixelPoint;
    std::cout << "Pixel xositi :" << PixelPos << std::endl;
  }
}

/*
 * Create lines and ticks for a grid in engineering units.
 */
auto vGridInPixels(float m2Pixel,               //!<
                   float GridXLowerLeft = -2.f, //!<
                   float GridYLowerLeft = -3.f, //!<
                   float GridLength = 6.f,      //!<
                   float GridHeigth = 6.f,      //!<
                   float TickDistance = 0.1f    //!<
                   ) -> std::vector<pixel_pos> {

  std::vector<pixel_pos> Result{};

  // ---
  // NOTE: Make and draw a grid.
  // ---
  struct grid_point {
    float fromX{};
    float fromY{};
    float toX{};
    float toY{};
  };

  const float NumTicks = GridLength / TickDistance;
  std::vector<grid_point> vGridPoint{};

  //!< Vertical left
  vGridPoint.push_back(grid_point{GridXLowerLeft, GridYLowerLeft,
                                  GridXLowerLeft, GridYLowerLeft + GridHeigth});

  //!< Horizontal lower
  vGridPoint.push_back(grid_point{GridXLowerLeft, GridYLowerLeft,
                                  GridXLowerLeft + GridLength, GridYLowerLeft});

  //!< Vertical rigth
  vGridPoint.push_back(grid_point{GridXLowerLeft + GridLength, GridYLowerLeft,
                                  GridXLowerLeft + GridLength,
                                  GridYLowerLeft + GridHeigth});

  //!< Horizontal upper
  vGridPoint.push_back(grid_point{GridXLowerLeft, GridYLowerLeft + GridHeigth,
                                  GridXLowerLeft + GridLength,
                                  GridYLowerLeft + GridHeigth});

  //!< Center Horizontal
  vGridPoint.push_back(grid_point{
      GridXLowerLeft, GridYLowerLeft + (GridHeigth / 2.f),
      GridXLowerLeft + GridLength, GridYLowerLeft + (GridHeigth / 2.f)});

  //!< Center Vertical
  vGridPoint.push_back(grid_point{
      GridXLowerLeft + GridLength / 2.f, GridYLowerLeft,
      GridXLowerLeft + GridLength / 2.f, GridYLowerLeft + GridHeigth});

  // ---
  // NOTE: Create ticks along the horizontal axis.
  // ---
  for (size_t Idx = 0; Idx < int(NumTicks); ++Idx) {
    float const PosX0 = GridXLowerLeft + float(Idx) * TickDistance;
    float const PosX1 = PosX0;
    float const PosY0 = GridYLowerLeft + GridHeigth / 2.f;
    float const PosY1 = PosY0 + TickDistance / 2.f;
    vGridPoint.push_back(grid_point{PosX0, PosY0, PosX1, PosY1});
  }

  // ---
  // NOTE: Create ticks along the vertical axis.
  // ---
  for (size_t Idx = 0; Idx < int(NumTicks); ++Idx) {
    float const PosX0 = GridXLowerLeft + GridLength / 2.f;
    float const PosX1 = PosX0 + TickDistance / 2.f;
    float const PosY0 = GridYLowerLeft + float(Idx) * TickDistance;
    float const PosY1 = PosY0;
    vGridPoint.push_back(grid_point{PosX0, PosY0, PosX1, PosY1});
  }

  for (auto Elem : vGridPoint) {
    Result.push_back(Conv2Pix(Elem.toX, Elem.toY, m2Pixel, m2Pixel));
    Result.push_back(Conv2Pix(Elem.fromX, Elem.fromY, m2Pixel, m2Pixel));
  }

  return Result;
};
//----------------------------------------------------------------------------------
// Main Enry Point
//----------------------------------------------------------------------------------
int main() {

  TestComputeGridPosition();
  Test3dCalucations();
  Test3dScreenCalculations();
  return 0;

  data Data{};
  auto pData = &Data;

  // Initialization
  //--------------------------------------------------------------------------------------
  InitWindow(Data.screenWidth, Data.screenHeight,
             "Fourier terms on a square wave");

#if defined(PLATFORM_WEB)
  emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
  SetTargetFPS(120); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  Data.vGridLines = vGridInPixels(Data.m2Pixel);

  int constexpr NumTerms = 5;
  InitFourierSquareWave(Data, NumTerms);

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    DrawFPS(10, 10);
    Data.dt = 1. / 60.f; // / GetFPS();
    if (!Data.StopUpdate)
      Data.t = GetTime();
    Data.Key = GetKeyPressed();

    UpdateDrawFrame(pData);
  }
#endif

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------

  return 0;
}

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
void UpdateDrawFrame(data *pData) {
  // Update
  //----------------------------------------------------------------------------------
  // TODO: Update your variables here
  //----------------------------------------------------------------------------------

  // Draw
  //----------------------------------------------------------------------------------
  BeginDrawing();

  ClearBackground(RAYWHITE);

  DrawText(
      std::string("Use arrow keys. Zoom: " + std::to_string(pData->m2Pixel))
          .c_str(),
      140, 10, 20, BLUE);
  DrawText(std::string("Num terms: " + std::to_string(pData->n) +
                       ". Key:" + std::to_string(pData->KeyPrv))
               .c_str(),
           140, 40, 20, BLUE);

  bool InputChanged{};

  if (KEY_DOWN == pData->Key) {
    pData->m2Pixel -= 10.0;
    InputChanged = true;
  }
  if (KEY_UP == pData->Key) {
    pData->m2Pixel += 10.0;
    InputChanged = true;
  }
  if (KEY_LEFT == pData->Key) {
    --pData->n;
    InputChanged = true;
  }
  if (KEY_RIGHT == pData->Key) {
    ++pData->n;
    InputChanged = true;
  }
  if (KEY_SPACE == pData->Key) {
    InputChanged = true;
    pData->StopUpdate = !pData->StopUpdate;
  }

  if (pData->Key)
    pData->KeyPrv = pData->Key;

  auto const Frequency = 1.0; /// pData->dt;
  auto const Omegat = M_2_PI * Frequency * pData->t;

  DrawText(std::to_string(pData->t).c_str(), 10, 50, 20, Fade(GREEN, 0.3f));
  DrawText(std::to_string(Frequency).c_str(), 10, 80, 20, Fade(GREEN, 0.3f));

  // ---
  // NOTE: The circle defining the base of the Fourier series.
  //       Input here is wt (that is omega * t, which is 2*pi*f*t)
  // ---
  float constexpr C0PosX = -5.f;
  float constexpr C0PosY = 0.f;
  float const C0Radius = pData->vSquareWaveElems.at(0).Amplitude;

  float DrawStartX = C0PosX + pData->X;
  float DrawStartY = C0PosY + pData->Y;

  pData->Fx = FuncFourierSquareWave(pData->vSquareWaveElems, Omegat);

  // ---
  // NOTE: Accumulate the X and Y terms in order to display a circle for each
  // term.
  // ---
  float AccX{};
  float AccY{};

  // ---
  // NOTE: Draw a circle for each term in the Fourier series.
  // ---
  for (std::size_t Idx = 0; Idx < pData->vSquareWaveElems.size(); ++Idx) {
    auto const &E = pData->vSquareWaveElems[Idx];
    auto const Radius = E.Amplitude;

    AccX += E.Xn;
    AccY += E.Yn;

    if (!Idx) {
      auto const PixelPos =
          Conv2Pix(C0PosX, C0PosY, pData->m2Pixel, pData->m2Pixel);
      DrawCircleLines(PixelPos.X, PixelPos.Y, Radius * pData->m2Pixel,
                      Fade(BLUE, 0.3f));
    } else {
      auto const FourierTerm = Conv2Pix(C0PosX + AccX, C0PosY + AccY,
                                        pData->m2Pixel, pData->m2Pixel);
      DrawCircleLines(FourierTerm.X, FourierTerm.Y, Radius * pData->m2Pixel,
                      Fade(BLUE, 0.3f));

      auto const CurrCircleLine =
          Conv2Pix(C0PosX + AccX + Radius * std::sin(E.Theta),
                   C0PosY + AccY + Radius * std::cos(E.Theta), pData->m2Pixel,
                   pData->m2Pixel);
      DrawLine(CurrCircleLine.X, CurrCircleLine.Y, FourierTerm.X, FourierTerm.Y,
               Fade(BLACK, 1.0f));
    }
  }

  // ---
  // NOTE: Draw the connection line from the circle to the end of the plot.
  // ---
  auto const IndLinePos1 = Conv2Pix(AccX, AccY, pData->m2Pixel, pData->m2Pixel);
  auto const IndLinePos2 = Conv2Pix(C0PosX + C0Radius * 1.2f + pData->Time,
                                    AccY, pData->m2Pixel, pData->m2Pixel);

  pData->Time += 1.0 / pData->m2Pixel;

  if (InputChanged ||
      (pData->Time > (GetScreenWidth() - IndLinePos1.X) / pData->m2Pixel)) {
    pData->Time = 0.0;
    pData->vPixelPos.clear();
    pData->vGridLines = vGridInPixels(pData->m2Pixel);
    InitFourierSquareWave(*pData, pData->n);
    return;
  }

  auto const DrawStartPx =
      Conv2Pix(C0PosX + AccX, C0PosY + AccY, pData->m2Pixel, pData->m2Pixel);
  auto const C0PosPx = Conv2Pix(C0PosX, C0PosY, pData->m2Pixel, pData->m2Pixel);

  DrawLine(C0PosPx.X, C0PosPx.Y, DrawStartPx.X, DrawStartPx.Y,
           Fade(BLACK, 1.0f));
  DrawLine(DrawStartPx.X, DrawStartPx.Y, IndLinePos2.X, IndLinePos2.Y,
           Fade(RED, 1.0f));

  pData->vPixelPos.push_back(IndLinePos2);
  for (auto const &CurvePoint : pData->vPixelPos) {
    DrawPixel(CurvePoint.X, CurvePoint.Y, RED);
  }

  for (size_t Idx = 0; Idx < pData->vGridLines.size(); Idx += 2) {
    auto const &Elem0 = pData->vGridLines[Idx];
    auto const &Elem1 = pData->vGridLines[Idx + 1];
    DrawLine(Elem0.X, Elem0.Y, Elem1.X, Elem1.Y, Fade(VIOLET, 1.0f));
  }

  EndDrawing();
  //----------------------------------------------------------------------------------
}
