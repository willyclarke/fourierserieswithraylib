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
// @ m2P - meter to pixel conversion factor
// @ Height, Width - Canvas dimension
pixel_pos Conv2Pix(float X, float Y, //!<
                   float m2P         //!<
) {
  pixel_pos Result{};
  Result.X = int(X * m2P) + (GetScreenWidth() >> 1);
  Result.Y = int(-Y * m2P) + (GetScreenHeight() >> 1);
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
  float dt{};
  float t{};
  std::vector<pixel_pos> vPixelPos{};
  std::vector<square_wave_elem> vSquareWaveElems{};
  std::vector<pixel_pos> vGridLines{};
};

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
  constexpr float FourOverPI = 4.0 / M_PI;
  return FourOverPI / (2.0 * float(nthterm) - 1.0);
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
    auto const Angle{(2.0 * n - 1.0) * Omegat};
    E.Yn = E.Amplitude * std::sin((Angle));
    E.Xn = E.Amplitude * std::cos(Angle);
    E.Theta = Omegat;
    Sum += E.Yn;
    n += 1.0;
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
    Result.push_back(Conv2Pix(Elem.toX, Elem.toY, m2Pixel));
    Result.push_back(Conv2Pix(Elem.fromX, Elem.fromY, m2Pixel));
  }

  return Result;
};
//----------------------------------------------------------------------------------
// Main Enry Point
//----------------------------------------------------------------------------------
int main() {
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
  float constexpr C0PosX = -3.f;
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
      auto const PixelPos = Conv2Pix(C0PosX, C0PosY, pData->m2Pixel);
      DrawCircleLines(PixelPos.X, PixelPos.Y, Radius * pData->m2Pixel,
                      Fade(BLUE, 0.3f));
    } else {
      auto const FourierTerm =
          Conv2Pix(C0PosX + AccX, C0PosY + AccY, pData->m2Pixel);
      DrawCircleLines(FourierTerm.X, FourierTerm.Y, Radius * pData->m2Pixel,
                      Fade(BLUE, 0.3f));

      auto const CurrCircleLine =
          Conv2Pix(C0PosX + AccX + Radius * std::sin(E.Theta),
                   C0PosY + AccY + Radius * std::cos(E.Theta), pData->m2Pixel);
      DrawLine(CurrCircleLine.X, CurrCircleLine.Y, FourierTerm.X, FourierTerm.Y,
               Fade(BLACK, 1.0f));
    }
  }

  // ---
  // NOTE: Draw the connection line from the circle to the end of the plot.
  // ---
  auto const IndLinePos1 = Conv2Pix(AccX, AccY, pData->m2Pixel);
  auto const IndLinePos2 =
      Conv2Pix(C0PosX + C0Radius * 1.2f + pData->Time, AccY, pData->m2Pixel);

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
      Conv2Pix(C0PosX + AccX, C0PosY + AccY, pData->m2Pixel);
  auto const C0PosPx = Conv2Pix(C0PosX, C0PosY, pData->m2Pixel);

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
