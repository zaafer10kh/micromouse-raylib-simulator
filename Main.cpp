#include "raylib.h"
#include "Robot.h"
#include "Maze.h"
#include <algorithm>

bool DrawSliderWithButtons(const char* label, float* value, float minVal, float maxVal, float step, int x, int y, int width) {
    int height = 20;
    int btnWidth = 24;
    int sliderWidth = width - (btnWidth * 2) - 10;
    
    Rectangle minusBtn = { (float)x, (float)y, (float)btnWidth, (float)height };
    Rectangle sliderTrack = { (float)x + btnWidth + 5, (float)y, (float)sliderWidth, (float)height };
    Rectangle plusBtn = { (float)x + btnWidth + 5 + sliderWidth + 5, (float)y, (float)btnWidth, (float)height };

    Vector2 mousePos = GetMousePosition();
    bool changed = false;

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (CheckCollisionPointRec(mousePos, minusBtn)) {
            *value = std::clamp(*value - step, minVal, maxVal);
            changed = true;
        }
        if (CheckCollisionPointRec(mousePos, plusBtn)) {
            *value = std::clamp(*value + step, minVal, maxVal);
            changed = true;
        }
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mousePos, sliderTrack)) {
        float norm = (mousePos.x - sliderTrack.x) / sliderTrack.width;
        norm = std::clamp(norm, 0.0f, 1.0f);
        *value = minVal + norm * (maxVal - minVal);
        changed = true;
    }

    // Render '-' button
    DrawRectangleRec(minusBtn, CheckCollisionPointRec(mousePos, minusBtn) ? DARKGRAY : GRAY);
    DrawRectangleLinesEx(minusBtn, 1, LIGHTGRAY);
    DrawText("-", x + 8, y + 2, 16, WHITE);

    // Render slider track & knob
    DrawRectangleRec(sliderTrack, DARKGRAY);
    DrawRectangleLinesEx(sliderTrack, 1, LIGHTGRAY);

    float knobX = sliderTrack.x + ((*value - minVal) / (maxVal - minVal)) * sliderTrack.width;
    DrawRectangle((int)knobX - 4, y - 2, 8, height + 4, SKYBLUE);

    // Render '+' button
    DrawRectangleRec(plusBtn, CheckCollisionPointRec(mousePos, plusBtn) ? DARKGRAY : GRAY);
    DrawRectangleLinesEx(plusBtn, 1, LIGHTGRAY);
    DrawText("+", (int)plusBtn.x + 6, y + 2, 16, WHITE);

    // Render value text below the slider row
    DrawText(TextFormat("%s: %.4f", label, *value), x, y + 24, 15, RAYWHITE);

    return changed;
}

int main() {
    const int mazeViewportWidth = 800;
    const int sidebarWidth = 300;
    const int screenWidth = mazeViewportWidth + sidebarWidth; // 1100 px total width
    const int screenHeight = 800;

    const float scale = (float)mazeViewportWidth / MAZE_SIZE_MM;

    InitWindow(screenWidth, screenHeight, "Micromouse - Simulator with Dedicated Control Sidebar");
    SetTargetFPS(60);

    Maze maze;
    Robot mouse;

    while (!WindowShouldClose()) {
        double dt = GetFrameTime();

        // Hotkey 'R': Reset maze & mouse position
        if (IsKeyPressed(KEY_R)) {
            maze.generateCompliantMaze();
            
            mouse.x = 96.0;
            mouse.y = 2796.0;
            mouse.theta = -M_PI / 2.0;

            mouse.cellX = 0;
            mouse.cellY = 0;
            mouse.orientation = NORTH;
            
            // Fixed state control for new robot engine
            mouse.physicalMotion.active = false;
            while(!mouse.commandQueue.empty()) mouse.commandQueue.pop();

            mouse.v = 0.0;
            mouse.w = 0.0;
            mouse.isCrashed = false;
            mouse.pidEnabled = false;

            mouse.resetFloodfillMemory();
        }

        // Hotkey 'P': Toggle Autonomous PID Centering
        if (IsKeyPressed(KEY_P)) {
            mouse.pidEnabled = !mouse.pidEnabled;
        }

        // Manual Keyboard Steering (Active when PID is OFF)
        if (!mouse.pidEnabled) {
            mouse.v = 0.0;
            mouse.w = 0.0;

            if (IsKeyDown(KEY_UP))    mouse.v = 300.0;
            if (IsKeyDown(KEY_DOWN))  mouse.v = -300.0;
            if (IsKeyDown(KEY_LEFT))  mouse.w = -3.14159; // Counter-Clockwise (Left)
            if (IsKeyDown(KEY_RIGHT)) mouse.w = 3.14159;  // Clockwise (Right)
        }

        mouse.update(dt, maze);

        BeginDrawing();
            ClearBackground(BLACK);

            // --- 1. MAZE SIMULATOR VIEWPORT ---
            maze.draw(scale);
            mouse.draw(scale);

            // --- 2. RIGHT SIDEBAR PANEL ---
            int sideX = mazeViewportWidth + 15;
            
            // Sidebar Background & Divider
            DrawRectangle(mazeViewportWidth, 0, sidebarWidth, screenHeight, Color{ 22, 22, 26, 255 });
            DrawLine(mazeViewportWidth, 0, mazeViewportWidth, screenHeight, DARKGRAY);

            int currentY = 20;

            // Header & Status
            DrawText("CONTROL PANEL", sideX, currentY, 20, GOLD);
            currentY += 30;

            DrawText("Mode:", sideX, currentY, 16, LIGHTGRAY);
            DrawText(mouse.pidEnabled ? "AUTONOMOUS PID" : "MANUAL DRIVE", sideX + 55, currentY, 16, mouse.pidEnabled ? SKYBLUE : YELLOW);
            currentY += 25;

            DrawText("Status:", sideX, currentY, 16, LIGHTGRAY);
            DrawText(mouse.isCrashed ? "CRASHED" : "RUNNING", sideX + 65, currentY, 16, mouse.isCrashed ? RED : GREEN);
            currentY += 35;

            // Divider Line
            DrawLine(sideX, currentY, sideX + sidebarWidth - 30, currentY, DARKGRAY);
            currentY += 15;

            // --- ALL 5 SENSOR READINGS ---
            DrawText("SENSOR READINGS", sideX, currentY, 18, LIME);
            currentY += 25;

            const char* sensorLabels[5] = {
                "Far-Left  (-45°)",
                "Left      (-90°)",
                "Front       (0°)",
                "Right     (+90°)",
                "Far-Right (+45°)"
            };

            for (size_t i = 0; i < mouse.sensors.size(); ++i) {
                DrawText(TextFormat("%s:", sensorLabels[i]), sideX, currentY, 14, RAYWHITE);
                DrawText(TextFormat("%.1f mm", mouse.sensors[i].distance), sideX + 160, currentY, 14, SKYBLUE);
                currentY += 20;
            }

            currentY += 15;
            // Divider Line
            DrawLine(sideX, currentY, sideX + sidebarWidth - 30, currentY, DARKGRAY);
            currentY += 15;

            // --- PID TUNING SLIDERS ---
            DrawText("PID GAIN TUNING", sideX, currentY, 18, GOLD);
            currentY += 30;

            int sliderWidth = 250;
            DrawSliderWithButtons("Kp", &mouse.Kp, 0.000f, 0.200f, 0.002f, sideX, currentY, sliderWidth);
            currentY += 55;

            DrawSliderWithButtons("Ki", &mouse.Ki, 0.000f, 0.050f, 0.0005f, sideX, currentY, sliderWidth);
            currentY += 55;

            DrawSliderWithButtons("Kd", &mouse.Kd, 0.000f, 0.100f, 0.001f, sideX, currentY, sliderWidth);
            currentY += 65;

            // Divider Line
            DrawLine(sideX, currentY, sideX + sidebarWidth - 30, currentY, DARKGRAY);
            currentY += 20;

            // --- CONTROLS / HOTKEYS GUIDE ---
            DrawText("HOTKEYS & CONTROLS", sideX, currentY, 16, LIGHTGRAY);
            currentY += 25;
            DrawText("• Arrow Keys : Steering (Manual)", sideX, currentY, 13, GRAY);
            currentY += 20;
            DrawText("• 'P' Key    : Toggle PID Mode", sideX, currentY, 13, GRAY);
            currentY += 20;
            DrawText("• 'R' Key    : Reset Maze & Mouse", sideX, currentY, 13, GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}