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
struct data {
  int screenWidth = 800;
  int screenHeight = 450;
  int Key{};
  int KeyPrv{};
  Vector3 startPos{};
  Vector3 endPos{};
  double X{};
  double Y{};
  double XAcc{};
  double m2Pixel{100}; //!< Meter 2 pixel conversion - multiplicator.
  double dt{};
  double t{};
  std::vector<pixel_pos> vPixelPos{};
};

//----------------------------------------------------------------------------------
// Module Functions Declaration
//----------------------------------------------------------------------------------
void UpdateDrawFrame(data *Data); // Update and Draw one frame

//----------------------------------------------------------------------------------
// Main Enry Point
//----------------------------------------------------------------------------------
int main() {
  data Data{};
  auto pData = &Data;

  // Initialization
  //--------------------------------------------------------------------------------------
  InitWindow(Data.screenWidth, Data.screenHeight,
             "raylib [core] example - basic window");

#if defined(PLATFORM_WEB)
  emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  Data.startPos = {0.f, 0.f, 0.f};
  Data.endPos = {12.f, 0.f, 0.f};

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

  // DrawText("Xongrats! You created your first window!", 140, 10, 20, BLUE);
  DrawText(std::to_string(pData->KeyPrv).c_str(), 140, 10, 20, BLUE);
  DrawText(std::to_string(pData->m2Pixel).c_str(), 140, 40, 20, BLUE);
  if (264 == pData->Key) {
    pData->m2Pixel -= 10.0;
  }
  if (265 == pData->Key) {
    pData->m2Pixel += 10.0;
  }
  if (pData->Key)
    pData->KeyPrv = pData->Key;

  // DrawRectangleGradientH(pData->screenWidth / 4 * 2 - 60, 100, 120, 60, BLUE,
  //                        MAROON);
  DrawLine(560, 0, 560, GetScreenHeight(), Fade(LIGHTGRAY, 0.6f));
  // DrawRectangle(560, 0, GetScreenWidth() - 500, GetScreenHeight(),
  //               Fade(LIGHTGRAY, 0.3f * pData->X));
  // DrawLine3D(pData->startPos, pData->endPos, LIME);

  DrawText(std::to_string(pData->t).c_str(), 10, 50, 20, Fade(LIGHTGRAY, 0.3f));
  DrawText(std::to_string(pData->dt).c_str(), 10, 80, 20,
           Fade(LIGHTGRAY, 0.3f));

  pData->X = std::cos(2. * M_PI * 1. / pData->dt * pData->t);
  pData->Y = std::sin(2. * M_PI * 1. / pData->dt * pData->t);

  DrawText(std::to_string(pData->X).c_str(), 10, 110, 20, Fade(GREEN, 0.3f));
  DrawText(std::to_string(pData->Y).c_str(), 10, 140, 20, Fade(GREEN, 0.3f));

  float constexpr CirclePosX = -3.f;
  float constexpr CirclePosY = 0.f;
  float constexpr RADIUS = 1.f;
  float DrawX = CirclePosX + pData->X;
  float DrawY = CirclePosY + pData->Y;
  auto const FxPx = Conv2Pix(DrawX, DrawY, pData->m2Pixel);
  auto const CirclePosPx = Conv2Pix(CirclePosX, CirclePosY, pData->m2Pixel);

  // ---
  // NOTE: Draw the connection line from the circle to the end of the plot.
  // ---
  auto const IndLinePos1 = Conv2Pix(pData->X, pData->Y, pData->m2Pixel);
  auto const IndLinePos2 = Conv2Pix(CirclePosX + RADIUS * 1.2f + pData->XAcc,
                                    pData->Y, pData->m2Pixel);

  pData->XAcc += 1.0 / pData->m2Pixel;
  if (pData->XAcc > (GetScreenWidth() >> 1) / pData->m2Pixel) {
    pData->XAcc = 0.0;
    pData->vPixelPos.clear();
  }

  DrawText(std::to_string(FxPx.X).c_str(), 10, 170, 20, Fade(GREEN, 0.3f));
  DrawText(std::to_string(FxPx.Y).c_str(), 10, 200, 20, Fade(GREEN, 0.3f));

  DrawLine(CirclePosPx.X, CirclePosPx.Y, FxPx.X, FxPx.Y, Fade(BLACK, 1.0f));
  DrawLine(FxPx.X, FxPx.Y, IndLinePos2.X, IndLinePos2.Y, Fade(RED, 1.0f));

  pData->vPixelPos.push_back(IndLinePos2);
  for (auto const &CurvePoint : pData->vPixelPos) {
    DrawPixel(CurvePoint.X, CurvePoint.Y, RED);
  }

  DrawCircleLines(CirclePosPx.X, CirclePosPx.Y, RADIUS * pData->m2Pixel,
                  Fade(BLUE, 0.3f));

  EndDrawing();
  //----------------------------------------------------------------------------------
}
