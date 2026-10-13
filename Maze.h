#ifndef MAZE_H
#define MAZE_H

#include "raylib.h"
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

const int MAZE_CELLS = 16;
const float CELL_PITCH_MM = 180.0f; // Center-to-center cell pitch (18.0 cm)
const float WALL_THICKNESS_MM = 12.0f; // Physical wall thickness (1.2 cm)
const float MAZE_SIZE_MM = (MAZE_CELLS * CELL_PITCH_MM) + WALL_THICKNESS_MM; // 2892.0 mm (289.2 cm)

struct WallRect {
    Rectangle rect; // Physical 2D bounding box (mm)
};

struct Cell {
    bool north = true;
    bool south = true;
    bool east  = true;
    bool west  = true;
    bool visited = false;
};

class Maze {
public:
    Cell grid[MAZE_CELLS][MAZE_CELLS];
    std::vector<WallRect> walls;

    Maze() {
        generateCompliantMaze();
    }

    void generateCompliantMaze() {
        for (int x = 0; x < MAZE_CELLS; ++x) {
            for (int y = 0; y < MAZE_CELLS; ++y) {
                grid[x][y] = Cell();
            }
        }

        std::srand((unsigned int)std::time(NULL));
        carvePassages(0, 0);

        // --- MICROMOUSE RULES ---
        // Start cell (0,0) - Only North exit
        grid[0][0].south = true;
        grid[0][0].west  = true;
        grid[0][0].east  = true;
        grid[0][0].north = false;
        grid[0][1].south = false;

        // Open Goal Center Zone: (7,7), (7,8), (8,7), (8,8)
        grid[7][7].east = false;  grid[8][7].west = false;
        grid[7][8].east = false;  grid[8][8].west = false;
        grid[7][7].north = false; grid[7][8].south = false;
        grid[8][7].north = false; grid[8][8].south = false;

        // Entrance to center
        grid[7][7].west = false;
        grid[6][7].east = false;

        buildSolidWalls();
    }

private:
    void carvePassages(int x, int y) {
        grid[x][y].visited = true;

        int dirs[4] = {0, 1, 2, 3};
        for (int i = 0; i < 4; ++i) {
            int r = i + std::rand() % (4 - i);
            std::swap(dirs[i], dirs[r]);
        }

        int dx[4] = {0, 1, 0, -1};
        int dy[4] = {1, 0, -1, 0};

        for (int i = 0; i < 4; ++i) {
            int d = dirs[i];
            int nx = x + dx[d];
            int ny = y + dy[d];

            if (nx >= 0 && nx < MAZE_CELLS && ny >= 0 && ny < MAZE_CELLS && !grid[nx][ny].visited) {
                if (d == 0)      { grid[x][y].north = false; grid[nx][ny].south = false; }
                else if (d == 1) { grid[x][y].east  = false; grid[nx][ny].west  = false; }
                else if (d == 2) { grid[x][y].south = false; grid[nx][ny].north = false; }
                else if (d == 3) { grid[x][y].west  = false; grid[nx][ny].east  = false; }

                carvePassages(nx, ny);
            }
        }
    }

    void buildSolidWalls() {
        walls.clear();
        float halfThick = WALL_THICKNESS_MM / 2.0f;

        for (int x = 0; x < MAZE_CELLS; ++x) {
            for (int y = 0; y < MAZE_CELLS; ++y) {
                // Cell center coordinates in mm
                float cx = (x * CELL_PITCH_MM) + (CELL_PITCH_MM / 2.0f) + halfThick;
                float cy = ((MAZE_CELLS - y - 1) * CELL_PITCH_MM) + (CELL_PITCH_MM / 2.0f) + halfThick;

                float x_left  = cx - (CELL_PITCH_MM / 2.0f) - halfThick;
                float x_right = cx + (CELL_PITCH_MM / 2.0f) - halfThick;
                float y_top   = cy - (CELL_PITCH_MM / 2.0f) - halfThick;
                float y_bot   = cy + (CELL_PITCH_MM / 2.0f) - halfThick;

                if (grid[x][y].north) {
                    walls.push_back({{ x_left, y_top, CELL_PITCH_MM + WALL_THICKNESS_MM, WALL_THICKNESS_MM }});
                }
                if (grid[x][y].south) {
                    walls.push_back({{ x_left, y_bot, CELL_PITCH_MM + WALL_THICKNESS_MM, WALL_THICKNESS_MM }});
                }
                if (grid[x][y].west) {
                    walls.push_back({{ x_left, y_top, WALL_THICKNESS_MM, CELL_PITCH_MM + WALL_THICKNESS_MM }});
                }
                if (grid[x][y].east) {
                    walls.push_back({{ x_right, y_top, WALL_THICKNESS_MM, CELL_PITCH_MM + WALL_THICKNESS_MM }});
                }
            }
        }
    }

public:
    void draw(float scale) const {
        for (const auto& w : walls) {
            Rectangle scaledRect = {
                w.rect.x * scale,
                w.rect.y * scale,
                w.rect.width * scale,
                w.rect.height * scale
            };
            DrawRectangleRec(scaledRect, WHITE);
        }

        // Draw goal area outline
        float halfThick = WALL_THICKNESS_MM / 2.0f;
        float goal_x = (7 * CELL_PITCH_MM + halfThick) * scale;
        float goal_y = ((MAZE_CELLS - 9) * CELL_PITCH_MM + halfThick) * scale;
        float goal_size = (2 * CELL_PITCH_MM) * scale;
        DrawRectangleLinesEx({goal_x, goal_y, goal_size, goal_size}, 2.0f, GREEN);
    }
};

#endif