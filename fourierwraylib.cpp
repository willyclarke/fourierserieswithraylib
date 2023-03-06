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
  double Amplitude{};
  double Yn{};
  double Xn{};
  double Theta{};
};

//------------------------------------------------------------------------------
struct data {
  int screenWidth = 1024;
  int screenHeight = 768;
  int Key{};
  int KeyPrv{};
  double X{};
  double Y{};
  double XAcc{};
  double Fx{};         //!< Fourier calculated series value.
  int n{1};            //!< Fourier series number of terms.
  double m2Pixel{100}; //!< Meter 2 pixel conversion - multiplicator.
  double dt{};
  double t{};
  std::vector<pixel_pos> vPixelPos{};
  std::vector<square_wave_elem> vSquareWaveElems{};
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
double FuncFourierSquareWaveAmplitude(int nthterm) {
  constexpr double FourOverPI = 4.0 / M_PI;
  return FourOverPI / (2.0 * double(nthterm) - 1.0);
}

//------------------------------------------------------------------------------
// @desc - Calculate each element of the Fourier square wave
// @NOTE: This function has side effect - it will update each element with the
//        current Theta angle and the value of each term.
// @return - Sum of n elements of the Fourier square wave.
double FuncFourierSquareWave(std::vector<square_wave_elem> &vSquareWaveElems,
                             double Omegat) {
  double Sum{};
  double n{1};
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
// @ input double t - time
// @ input n - number of terms
// @ output f(t) - at time t
// @ link :
// https://www.math.kit.edu/iana3/lehre/fourierana2014w/media/fstable141127.pdf
// @ function number 6
double FuncFourierSquareWave(double Theta, int N) {
  double Sum{};
  for (int n = 0; n < N; ++n) {
    Sum += std::sin((2.0 * double(n) - 1.0) * Theta) / (2.0 * double(n) - 1.0);
  }

  constexpr double FourOverPI = 4.0 / M_PI;
  double const Result = FourOverPI * Sum;

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
    Element.Amplitude = 4.0 / M_PI / (2.0 * double(Idx) - 1);
    Element.Yn = 0.0;
    Element.Theta = 0.0;
    Data.vSquareWaveElems.push_back(Element);
  }
}

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
  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  int constexpr NumTerms = 5;
  InitFourierSquareWave(Data, NumTerms);

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    DrawFPS(10, 10);
    Data.dt = 1. / 60.f; // / GetFPS();
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
  DrawText(std::string("Num terms: " + std::to_string(pData->n)).c_str(), 140,
           40, 20, BLUE);

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
  double AccX{};
  double AccY{};

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
    }
  }

  // ---
  // NOTE: Draw the connection line from the circle to the end of the plot.
  // ---
  auto const IndLinePos1 = Conv2Pix(AccX, AccY, pData->m2Pixel);
  auto const IndLinePos2 = Conv2Pix(C0PosX + C0Radius * 1.2f + pData->XAcc,
                                    pData->Fx, pData->m2Pixel);

  pData->XAcc += 1.0 / pData->m2Pixel;

  if (InputChanged ||
      (pData->XAcc > (GetScreenWidth() - IndLinePos1.X) / pData->m2Pixel)) {
    pData->XAcc = 0.0;
    pData->vPixelPos.clear();
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

  EndDrawing();
  //----------------------------------------------------------------------------------
}
