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

/**
 * Hold pixel position as integers, X and Y.
 */
struct pixel_pos {
  int X{};
  int Y{};
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
  float Xcalc{};
  float FofX{}; //!< Fourier calculated series value.
  int n{5};     //!< Fourier series number of terms.
  float dt{};
  float t{};
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

//----------------------------------------------------------------------------------
// Main Enry Point
//----------------------------------------------------------------------------------
int main(int argc, char *argv[]) {

  if (argc > 1) {
    es::TestHomogenousMatrix();
    es::Test3dCalucations();
    es::Test3dScreenCalculations();
  }

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

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    DrawFPS(10, 10);
    Data.dt = 1. / 60.f; // / GetFPS();
    if (!Data.StopUpdate)
      Data.t = GetTime();
    Data.Key = GetKeyPressed();

    UpdateDrawFrameFourier(pData);
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
    pData->Xcalc = 0.0;
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
                       ". Time:" + std::to_string(pData->Xcalc))
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

  auto Ft =
      Centre + es::Vector(Radius * cosf(Omegat), Radius * sinf(Omegat), 0.f);

  ldaDrawCircle(pData->Hep, Centre, Radius);

  // Draw the outer circle line
  ldaDrawLine(pData->Hep, Centre, Ft);

  // ---
  // Create the Fourier series.
  // ---
  auto Ftp = Ft;
  for (int Idx = 1; Idx < pData->n; ++Idx) {
    auto nthTerm = 1.f + Idx * 2.f;
    auto Ftn = Ftp + es::Vector(Radius / nthTerm * cosf(nthTerm * Omegat),
                                Radius / nthTerm * sinf(nthTerm * Omegat), 0.f);
    ldaDrawLine(pData->Hep, Ftp, Ftn);
    ldaDrawCircle(pData->Hep, Ftn, Radius / nthTerm);
    Ftp = Ftn;
  }

  auto GridStart = es::Point(0.f, 0.f, 0.f);
  pData->Xcalc += pData->dt;

  // Reset time - based on knowing that the default grid settings are from -4 to
  // +4. FIXME.
  if (pData->Xcalc > 4.f) {
    pData->Xcalc = -4.f;
    pData->vTrendPoints.clear();
  }

  auto AnimationPoint = GridStart + es::Vector(pData->Xcalc, Ftp.y, 0.f);

  // Draw the actual trend
  pData->vTrendPoints.push_back(AnimationPoint);
  for (auto E : pData->vTrendPoints) {
    ldaDrawPoint(pData->Hep, E, {pData->Hep.m0, pData->Hep.m5, 0.f, 0.f});
  }

  // Draw the inner circle line
  ldaDrawLine(pData->Hep, Ft, Ftp);
  // Draw the connecting line
  ldaDrawLine(pData->Hep, Ftp, AnimationPoint);

  EndDrawing();
}
