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

#include "engsupport.hpp"
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

/**
 * Hold pixel position as integers, X and Y.
 */
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
  bool ShowGrid{true};
  float X{};
  float Y{};
  float Time{};
  float Fx{}; //!< Fourier calculated series value.
  int n{1};   //!< Fourier series number of terms.
  float dt{};
  float t{};
  std::vector<pixel_pos> vPixelPos{};
  std::vector<square_wave_elem> vSquareWaveElems{};
  std::vector<Vector4> vTrendPoints{};
  std::vector<pixel_pos> vGridLines{};

  Matrix Hep{}; //!< Homogenous matrix for conversion from engineering space to
                //!< pixelspace.

  Vector4 vEngOffset{}; //!< Position of figure in engineering space.
  Vector4 vPixelsPerUnit{100.f, 100.f, 100.f, 0.f};
};

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
void UpdateDrawFrameFourier(data *Data); // Update and Draw one frame
void UpdateDrawFrame(data *Data);        // Update and Draw one frame
void UpdateDrawFrame2(data *Data);       // Update and Draw one frame

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

//------------------------------------------------------------------------------
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

/**
 * Initialize the matrix to convert from engineering basis to screen basis.
 * The screen center is used as the reference point.
 *
 * @OrigoScreen - the X, Y, Z values in engineering space at center of screen.
 * @vPixelsPerUnit - The number of pixels per unit, i.e 100 pixels equals 1m.
 * @ScreenCenterInPixels - The coordinates for the centre of the screen in
 * pixels.
 */
auto InitEng2PixelMatrix(Vector4 const &OrigoScreen,
                         Vector4 const &vPixelsPerUnit,
                         Vector4 const &ScreenPosInPixels) -> Matrix {

  // ---
  // Flip because pixel coord increases when moving down.
  // ---
  constexpr float Flip = -1.f;
  constexpr float NoFlip = 1.f;

  // ---
  // Create a Homogenous matrix that converts from engineering unit to screen.
  // ---
  auto Hes = es::I();
  Hes.m12 = ScreenPosInPixels.x + OrigoScreen.x * vPixelsPerUnit.x;
  Hes.m13 = ScreenPosInPixels.y + OrigoScreen.y * vPixelsPerUnit.y;
  Hes.m14 = ScreenPosInPixels.z + OrigoScreen.z * vPixelsPerUnit.z;

  // Flip and scale to pixel value.
  Hes.m0 = NoFlip * vPixelsPerUnit.x;
  Hes.m5 = Flip * vPixelsPerUnit.y;
  Hes.m10 = NoFlip * vPixelsPerUnit.z;

  return Hes;
}

/*
 * Create lines and ticks for a grid in engineering units.
 */
auto vGridInPixels(Matrix const &Hep, //!< Homogenous matrix from engineering to
                                      //!< pixel position.
                   float GridXLowerLeft = -4.f, //!<
                   float GridYLowerLeft = -3.f, //!<
                   float GridLength = 8.f,      //!<
                   float GridHeight = 6.f,      //!<
                   float TickDistance = 0.1f    //!<
                   ) -> std::vector<pixel_pos> {

  // ---
  // NOTE: Make and draw a grid.
  // ---
  struct grid_point {
    float fromX{};
    float fromY{};
    float toX{};
    float toY{};
  };

  const float NumTicksX = GridLength / TickDistance;
  const float NumTicksY = GridHeight / TickDistance;
  std::vector<grid_point> vGridPoint{};

  //!< Vertical left
  vGridPoint.push_back(grid_point{GridXLowerLeft, GridYLowerLeft,
                                  GridXLowerLeft, GridYLowerLeft + GridHeight});

  //!< Horizontal lower
  vGridPoint.push_back(grid_point{GridXLowerLeft, GridYLowerLeft,
                                  GridXLowerLeft + GridLength, GridYLowerLeft});

  //!< Vertical rigth
  vGridPoint.push_back(grid_point{GridXLowerLeft + GridLength, GridYLowerLeft,
                                  GridXLowerLeft + GridLength,
                                  GridYLowerLeft + GridHeight});

  //!< Horizontal upper
  vGridPoint.push_back(grid_point{GridXLowerLeft, GridYLowerLeft + GridHeight,
                                  GridXLowerLeft + GridLength,
                                  GridYLowerLeft + GridHeight});

  //!< Center Horizontal
  vGridPoint.push_back(grid_point{
      GridXLowerLeft, GridYLowerLeft + (GridHeight / 2.f),
      GridXLowerLeft + GridLength, GridYLowerLeft + (GridHeight / 2.f)});

  //!< Center Vertical
  vGridPoint.push_back(grid_point{
      GridXLowerLeft + GridLength / 2.f, GridYLowerLeft,
      GridXLowerLeft + GridLength / 2.f, GridYLowerLeft + GridHeight});

  // ---
  // NOTE: Create ticks along the horizontal axis.
  // ---
  for (size_t Idx = 0; Idx < int(NumTicksX); ++Idx) {
    float const PosX0 = GridXLowerLeft + float(Idx) * TickDistance;
    float const PosX1 = PosX0;
    float const PosY0 = GridYLowerLeft + GridHeight / 2.f;
    float const PosY1 = PosY0 + TickDistance / 2.f;
    vGridPoint.push_back(grid_point{PosX0, PosY0, PosX1, PosY1});
  }

  // ---
  // NOTE: Create ticks along the vertical axis.
  // ---
  for (size_t Idx = 0; Idx < int(NumTicksY); ++Idx) {
    float const PosX0 = GridXLowerLeft + GridLength / 2.f;
    float const PosX1 = PosX0 + TickDistance / 2.f;
    float const PosY0 = GridYLowerLeft + float(Idx) * TickDistance;
    float const PosY1 = PosY0;
    vGridPoint.push_back(grid_point{PosX0, PosY0, PosX1, PosY1});
  }

  std::vector<pixel_pos> Result{};
  for (auto Elem : vGridPoint) {
    auto const ToPixel = Hep * es::Point(Elem.toX, Elem.toY, 0.f);
    auto const FromPixel = Hep * es::Point(Elem.fromX, Elem.fromY, 0.f);
    Result.push_back({int(ToPixel.x), int(ToPixel.y)});
    Result.push_back({int(FromPixel.x), int(FromPixel.y)});
  }

  return Result;
};

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
//----------------------------------------------------------------------------------
// Main Enry Point
//----------------------------------------------------------------------------------
int main() {

  TestHomogenousMatrix();
  Test3dCalucations();
  Test3dScreenCalculations();

  data Data{};
  Data.vTrendPoints.reserve(size_t(Data.screenWidth));

  auto pData = &Data;

  // Initialization
  //--------------------------------------------------------------------------------------
  InitWindow(Data.screenWidth, Data.screenHeight,
             "Fourier terms on a square wave");

#if defined(PLATFORM_WEB)
  emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
  SetTargetFPS(60); // Set our game to run at 60 frames-per-second

  // ---
  // NOTE: Move all points around by setting the engineering offset which will
  // be added to the offset of the screen position in pixels.
  // ---
  Data.vEngOffset = es::Point(0.f, 0.f, 0.f);
  Data.vPixelsPerUnit = es::Point(100.f, 100.f, 0.f);

  Data.Hep = InitEng2PixelMatrix(
      Data.vEngOffset, Data.vPixelsPerUnit,
      {Data.screenWidth / 2.f, Data.screenHeight / 2.f, 0.f, 0.f});

  Data.vGridLines = vGridInPixels(Data.Hep);

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

    UpdateDrawFrameFourier(pData);
    // UpdateDrawFrame(pData);
  }
#endif

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------

  return 0;
}

/**
 * Keyboard input handling common to all the drawing routines.
 */
auto HandleKeyboardInput(data *pData) -> bool {
  bool InputChanged{};

  constexpr float MinPixelPerUnit = 50.f;

  if (pData->Key) {
    if (KEY_G == pData->Key) {
      pData->ShowGrid = !pData->ShowGrid;
      InputChanged = true;
    } else if (KEY_DOWN == pData->Key) {

      auto &vPPU = pData->vPixelsPerUnit;
      vPPU.x = std::max(vPPU.x - 10.f, MinPixelPerUnit);
      vPPU.y = std::max(vPPU.y - 10.f, MinPixelPerUnit);
      vPPU.z = std::max(vPPU.z - 10.f, MinPixelPerUnit);

      InputChanged = true;
    } else if (KEY_UP == pData->Key) {

      auto &vPPU = pData->vPixelsPerUnit;
      vPPU.x = std::max(vPPU.x + 10.f, MinPixelPerUnit);
      vPPU.y = std::max(vPPU.y + 10.f, MinPixelPerUnit);
      vPPU.z = std::max(vPPU.z + 10.f, MinPixelPerUnit);

      InputChanged = true;
    } else if (KEY_LEFT == pData->Key) {
      --pData->n;
      InputChanged = true;
    } else if (KEY_RIGHT == pData->Key) {
      ++pData->n;
      InputChanged = true;
    } else if (KEY_SPACE == pData->Key) {
      InputChanged = true;
      pData->StopUpdate = !pData->StopUpdate;
    }

    pData->KeyPrv = pData->Key;
  }

  if (InputChanged) {
    pData->Time = 0.0;
    pData->vPixelPos.clear();
    pData->vGridLines = vGridInPixels(pData->Hep);
    InitFourierSquareWave(*pData, pData->n);

    pData->Hep = InitEng2PixelMatrix(
        pData->vEngOffset, pData->vPixelsPerUnit,
        {pData->screenWidth / 2.f, pData->screenHeight / 2.f, 0.f, 0.f});
  }
  return InputChanged;
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

  DrawText(std::string("Use arrow keys. Zoom: " +
                       std::to_string(pData->vPixelsPerUnit.x))
               .c_str(),
           140, 10, 20, BLUE);
  DrawText(std::string("Num terms: " + std::to_string(pData->n) +
                       ". Key:" + std::to_string(pData->KeyPrv))
               .c_str(),
           140, 40, 20, BLUE);

  bool const InputChanged = HandleKeyboardInput(pData);

  auto const Frequency = 1.0; /// pData->dt;
  auto const Omegat = M_2_PI * Frequency * pData->t;

  DrawText(std::to_string(pData->t).c_str(), 10, 50, 20, Fade(GREEN, 0.3f));
  DrawText(std::to_string(Frequency).c_str(), 10, 80, 20, Fade(GREEN, 0.3f));

  // ---
  // NOTE: The circle defining the base of the Fourier series.
  //       Input here is wt (that is omega * t, which is 2*pi*f*t)
  // ---
  float constexpr C0PosX = -9.f;
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
      AccX = 0.f;
      auto const PixelPos = Conv2Pix(C0PosX, C0PosY, pData->vPixelsPerUnit.y,
                                     pData->vPixelsPerUnit.y);
      DrawCircleLines(PixelPos.X, PixelPos.Y, Radius * pData->vPixelsPerUnit.y,
                      Fade(BLUE, 0.3f));
    } else {
      auto const FourierTerm =
          Conv2Pix(C0PosX + AccX, C0PosY + AccY, pData->vPixelsPerUnit.y,
                   pData->vPixelsPerUnit.y);
      DrawCircleLines(FourierTerm.X, FourierTerm.Y,
                      Radius * pData->vPixelsPerUnit.y, Fade(BLUE, 0.3f));

      auto const CurrCircleLine =
          Conv2Pix(C0PosX + AccX + Radius * std::sin(E.Theta),
                   C0PosY + AccY + Radius * std::cos(E.Theta),
                   pData->vPixelsPerUnit.y, pData->vPixelsPerUnit.y);
      DrawLine(CurrCircleLine.X, CurrCircleLine.Y, FourierTerm.X, FourierTerm.Y,
               Fade(ORANGE, 1.0f));
    }
  }

  // ---
  // NOTE: Draw the connection line from the circle to the end of the plot.
  // ---
  auto const IndLinePos1 =
      Conv2Pix(AccX, AccY, pData->vPixelsPerUnit.y, pData->vPixelsPerUnit.y);
  auto const IndLinePos2 =
      Conv2Pix(C0PosX + C0Radius * 1.2f + pData->Time, AccY,
               pData->vPixelsPerUnit.y, pData->vPixelsPerUnit.y);

  pData->Time += 1.0 / pData->vPixelsPerUnit.y;

  if (InputChanged ||
      (pData->Time >
       (GetScreenWidth() - 0.95f * IndLinePos1.X) / pData->vPixelsPerUnit.y)) {
    pData->Time = 0.0;
    pData->vPixelPos.clear();
    pData->vGridLines = vGridInPixels(pData->Hep);
    InitFourierSquareWave(*pData, pData->n);
    return;
  }

  auto const DrawStartPx =
      Conv2Pix(C0PosX + AccX, C0PosY + AccY, pData->vPixelsPerUnit.y,
               pData->vPixelsPerUnit.y);
  auto const C0PosPx = Conv2Pix(C0PosX, C0PosY, pData->vPixelsPerUnit.y,
                                pData->vPixelsPerUnit.y);

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

/**
 * Second version of the drawing Functions
 */
void UpdateDrawFrame2(data *pData) {
  // Update
  //----------------------------------------------------------------------------------
  // TODO: Update your variables here
  //----------------------------------------------------------------------------------

  // Draw
  //----------------------------------------------------------------------------------
  BeginDrawing();

  ClearBackground(RAYWHITE);

  DrawText(std::string("Use arrow keys. Zoom: " +
                       std::to_string(pData->vPixelsPerUnit.x))
               .c_str(),
           140, 10, 20, BLUE);
  DrawText(std::string("Num terms: " + std::to_string(pData->n) +
                       ". Key:" + std::to_string(pData->KeyPrv))
               .c_str(),
           140, 40, 20, BLUE);

  bool const InputChanged = HandleKeyboardInput(pData);

  constexpr float MinPixelPerUnit = 50.f;

  if (InputChanged) {
    pData->Time = 0.0;
    pData->vPixelPos.clear();
    pData->vGridLines = vGridInPixels(pData->Hep);
    InitFourierSquareWave(*pData, pData->n);

    pData->Hep = InitEng2PixelMatrix(
        pData->vEngOffset, pData->vPixelsPerUnit,
        {pData->screenWidth / 2.f, pData->screenHeight / 2.f, 0.f, 0.f});

    return;
  }

  if (pData->ShowGrid) {
    for (size_t Idx = 0; Idx < pData->vGridLines.size(); Idx += 2) {
      auto const &Elem0 = pData->vGridLines[Idx];
      auto const &Elem1 = pData->vGridLines[Idx + 1];
      DrawLine(Elem0.X, Elem0.Y, Elem1.X, Elem1.Y, Fade(VIOLET, 1.0f));
    }
  }

  // ---
  // NOTE: Lamda to draw a point.
  // ---
  auto DrawPoint = [](Matrix const &Hep, Vector4 const &P,
                      Vector4 const &m2Pixel, bool Print = false) -> void {
    auto CurvePoint = Hep * P;
    DrawPixel(CurvePoint.x, CurvePoint.y, RED);
    constexpr float Radius = 0.1f;
    DrawCircleLines(CurvePoint.x, CurvePoint.y, Radius * m2Pixel.x,
                    Fade(BLUE, 0.3f));
    DrawLine(CurvePoint.x, CurvePoint.y, 0, 0, BLUE);
    if (Print)
      DrawText(std::string("CurvePoint x/y: " + std::to_string(CurvePoint.x) +
                           " / " + std::to_string(CurvePoint.y))
                   .c_str(),
               140, 70, 20, BLUE);
  };

  DrawPoint(pData->Hep, es::Point(0.f, 0.f, 0.f), pData->vPixelsPerUnit, true);
  DrawPoint(pData->Hep, es::Point(1.f, 1.f, 0.f), pData->vPixelsPerUnit);
  DrawPoint(pData->Hep, es::Point(1.f, -1.f, 0.f), pData->vPixelsPerUnit);
  DrawPoint(pData->Hep, es::Point(-1.f, 1.f, 0.f), pData->vPixelsPerUnit);
  DrawPoint(pData->Hep, es::Point(-1.f, -1.f, 0.f), pData->vPixelsPerUnit);

  EndDrawing();
}

/**
 */
void UpdateDrawFrameFourier(data *pData) {
  BeginDrawing();
  ClearBackground(RAYWHITE);

  DrawText(std::string("Use arrow keys. Zoom: " +
                       std::to_string(pData->vPixelsPerUnit.x))
               .c_str(),
           140, 10, 20, BLUE);
  DrawText(std::string("Num terms: " + std::to_string(pData->n) +
                       ". Key:" + std::to_string(pData->KeyPrv) +
                       ". Time:" + std::to_string(pData->Time))
               .c_str(),
           140, 40, 20, BLUE);

  bool const InputChanged = HandleKeyboardInput(pData);

  if (pData->ShowGrid) {
    for (size_t Idx = 0; Idx < pData->vGridLines.size(); Idx += 2) {
      auto const &Elem0 = pData->vGridLines[Idx];
      auto const &Elem1 = pData->vGridLines[Idx + 1];
      DrawLine(Elem0.X, Elem0.Y, Elem1.X, Elem1.Y, Fade(VIOLET, 1.0f));
    }
  }

  // ---
  // NOTE: Lamda to draw a point.
  // ---
  auto ldaDrawPoint = [](Matrix const &Hep, Vector4 const &P,
                         Vector4 const &m2Pixel, bool Print = false) -> void {
    auto CurvePoint = Hep * P;
    DrawPixel(CurvePoint.x, CurvePoint.y, RED);
    constexpr float Radius = 0.01f;
    DrawCircleLines(CurvePoint.x, CurvePoint.y, Radius * m2Pixel.x,
                    Fade(BLUE, 0.3f));
    if (Print) {
      DrawLine(CurvePoint.x, CurvePoint.y, 0, 0, BLUE);
      DrawText(std::string("CurvePoint x/y: " + std::to_string(CurvePoint.x) +
                           " / " + std::to_string(CurvePoint.y))
                   .c_str(),
               140, 70, 20, BLUE);
    }
  };

  auto ldaDrawCircle = [](Matrix const &Hep, Vector4 const &Centre,
                          float Radius, Color Col = BLUE) -> void {
    auto CurvePoint = Hep * Centre;
    DrawCircleLines(CurvePoint.x, CurvePoint.y, Radius * Hep.m5,
                    Fade(Col, 0.3f));
  };

  auto ldaDrawLine = [](Matrix const &Hep, Vector4 const &From,
                        Vector4 const &To, Color Col = BLUE) -> void {
    auto F = Hep * From;
    auto T = Hep * To;
    DrawLine(F.x, F.y, T.x, T.y, BLUE);
  };

  auto const Frequency = 2.0;
  auto const Omegat = M_2_PI * Frequency * pData->t;
  auto const Radius = 4.f / M_PI;
  auto Centre = es::Point(-5.f, 0.f, 0.f);

  auto Ft = Centre + es::Vector(Radius * std::cosf(Omegat),
                                Radius * std::sinf(Omegat), 0.f);

  ldaDrawCircle(pData->Hep, Centre, Radius);

  // ---
  // Create the Fourier series.
  // ---
  auto Ftp = Ft;
  for (int Idx = 1; Idx < pData->n; ++Idx) {
    auto nthTerm = 1.f + Idx * 2.f;
    auto Ftn =
        Ftp + es::Vector(Radius / nthTerm * std::cosf(nthTerm * Omegat),
                         Radius / nthTerm * std::sinf(nthTerm * Omegat), 0.f);
    ldaDrawLine(pData->Hep, Ftp, Ftn);
    ldaDrawCircle(pData->Hep, Ftn, Radius / nthTerm);
    Ftp = Ftn;
  }

  auto GridStart = es::Point(0.f, 0.f, 0.f);
  pData->Time += pData->dt;

  // Reset time - based on knowing that the default grid settings are from -4 to
  // +4. FIXME.
  if (pData->Time > 4.f) {
    pData->Time = -4.f;
    pData->vTrendPoints.clear();
  }

  auto AnimationPoint = GridStart + es::Vector(pData->Time, Ftp.y, 0.f);

  // Draw the actual trend
  pData->vTrendPoints.push_back(AnimationPoint);
  for (auto E : pData->vTrendPoints) {
    ldaDrawPoint(pData->Hep, E, {pData->Hep.m0, pData->Hep.m5, 0.f, 0.f});
  }

  // Draw the connecting line
  ldaDrawLine(pData->Hep, Ftp, AnimationPoint);

  EndDrawing();
}
