#ifndef ROBOT_H
#define ROBOT_H

#include "raylib.h"
#include "Maze.h"
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

enum CommandType {
    CMD_MOVE_FORWARD_ONE_CELL,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_TURN_180
};

struct Sensor {
    float angleOffset;   
    float maxRange;      
    float distance;      
    Vector2 hitPoint;    
    Vector2 localMount;  
};

struct MotionState {
    bool active = false;
    CommandType currentCmd;
    double startX = 0.0;
    double startY = 0.0;
    double targetAngle = 0.0;
    int nextCellX = 0;
    int nextCellY = 0;
    Direction nextOrientation;
};

class Robot {
public:
    double x = 96.0;       
    double y = 2796.0;     
    double theta = -M_PI / 2.0; // Pointing North (-Y direction in Raylib)

    double v = 0.0;        
    double w = 0.0;        

    float width = 80.0f;   
    float length = 100.0f; 

    bool pidEnabled = false;
    bool isCrashed = false;

    std::vector<Sensor> sensors;

    int cellX = 0;
    int cellY = 0;
    Direction orientation = NORTH;

    Cell knownMaze[MAZE_CELLS][MAZE_CELLS];
    int floodDist[MAZE_CELLS][MAZE_CELLS];

    std::queue<CommandType> commandQueue;
    MotionState physicalMotion;

    // Tuned default gains for normalized error [-1.0, 1.0]
    float Kp = 0.400f;
    float Ki = 0.005f;
    float Kd = 0.060f;

    double errorSum = 0.0;
    double lastError = 0.0;

    Robot() {
        float halfL = length / 2.0f;
        float halfW = width / 2.0f;

        // 5 Emitter Sensors
        sensors.push_back({ (float)(-M_PI / 4.0), 300.0f, 300.0f, {0,0}, {  halfL, -halfW } }); // Far-Left (-45°)
        sensors.push_back({ (float)(-M_PI / 2.0), 300.0f, 300.0f, {0,0}, {   0.0f, -halfW } }); // Left (-90°)
        sensors.push_back({ 0.0f,                 300.0f, 300.0f, {0,0}, {  halfL,   0.0f } }); // Front (0°)
        sensors.push_back({ (float)(M_PI / 2.0),  300.0f, 300.0f, {0,0}, {   0.0f,  halfW } }); // Right (+90°)
        sensors.push_back({ (float)(M_PI / 4.0),  300.0f, 300.0f, {0,0}, {  halfL,  halfW } }); // Far-Right (+45°)

        resetFloodfillMemory();
    }

    void resetFloodfillMemory() {
        for (int x = 0; x < MAZE_CELLS; ++x) {
            for (int y = 0; y < MAZE_CELLS; ++y) {
                knownMaze[x][y].north = false;
                knownMaze[x][y].south = false;
                knownMaze[x][y].east  = false;
                knownMaze[x][y].west  = false;
                knownMaze[x][y].visited = false;
            }
        }

        for (int i = 0; i < MAZE_CELLS; ++i) {
            knownMaze[i][0].south = true;
            knownMaze[i][MAZE_CELLS - 1].north = true;
            knownMaze[0][i].west = true;
            knownMaze[MAZE_CELLS - 1][i].east = true;
        }

        recalculateFloodfill();
    }

    void recalculateFloodfill() {
        for (int x = 0; x < MAZE_CELLS; ++x) {
            for (int y = 0; y < MAZE_CELLS; ++y) {
                floodDist[x][y] = 255;
            }
        }

        std::queue<std::pair<int, int>> q;
        int goals[4][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
        for (auto& g : goals) {
            floodDist[g[0]][g[1]] = 0;
            q.push({g[0], g[1]});
        }

        int dx[4] = {0, 1, 0, -1}; // N, E, S, W
        int dy[4] = {1, 0, -1, 0};

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();

            int currentD = floodDist[cx][cy];

            for (int d = 0; d < 4; ++d) {
                int nx = cx + dx[d];
                int ny = cy + dy[d];

                if (nx < 0 || nx >= MAZE_CELLS || ny < 0 || ny >= MAZE_CELLS) continue;

                bool wallBlocked = false;
                if (d == 0 && knownMaze[cx][cy].north) wallBlocked = true;
                if (d == 1 && knownMaze[cx][cy].east)  wallBlocked = true;
                if (d == 2 && knownMaze[cx][cy].south) wallBlocked = true;
                if (d == 3 && knownMaze[cx][cy].west)  wallBlocked = true;

                if (!wallBlocked && floodDist[nx][ny] == 255) {
                    floodDist[nx][ny] = currentD + 1;
                    q.push({nx, ny});
                }
            }
        }
    }

    void scanCellWalls() {
        float sideThreshold  = 85.0f; // Side clearance check
        float frontThreshold = 95.0f; // Front wall check

        bool hasLeft  = sensors[1].distance < sideThreshold;
        bool hasFront = sensors[2].distance < frontThreshold;
        bool hasRight = sensors[3].distance < sideThreshold;

        int frontDir = (int)orientation;
        int leftDir  = (orientation + 3) % 4;
        int rightDir = (orientation + 1) % 4;

        auto setWallSymmetric = [&](int x, int y, int dir, bool present) {
            if (x < 0 || x >= MAZE_CELLS || y < 0 || y >= MAZE_CELLS) return;

            if (dir == NORTH) {
                knownMaze[x][y].north = present;
                if (y + 1 < MAZE_CELLS) knownMaze[x][y + 1].south = present;
            }
            else if (dir == EAST) {
                knownMaze[x][y].east = present;
                if (x + 1 < MAZE_CELLS) knownMaze[x + 1][y].west = present;
            }
            else if (dir == SOUTH) {
                knownMaze[x][y].south = present;
                if (y - 1 >= 0) knownMaze[x][y - 1].north = present;
            }
            else if (dir == WEST) {
                knownMaze[x][y].west = present;
                if (x - 1 >= 0) knownMaze[x - 1][y].east = present;
            }
        };

        setWallSymmetric(cellX, cellY, frontDir, hasFront);
        setWallSymmetric(cellX, cellY, leftDir,  hasLeft);
        setWallSymmetric(cellX, cellY, rightDir, hasRight);

        knownMaze[cellX][cellY].visited = true;
        recalculateFloodfill();
    }

    double getCardinalAngle(Direction dir) const {
        switch (dir) {
            case NORTH: return -M_PI / 2.0;
            case EAST:  return 0.0;
            case SOUTH: return M_PI / 2.0;
            case WEST:  return M_PI;
        }
        return 0.0;
    }

    void runFloodfillBrain() {
        scanCellWalls();

        if (floodDist[cellX][cellY] == 0) return;

        int dx[4] = {0, 1, 0, -1}; 
        int dy[4] = {1, 0, -1, 0};

        int bestDir = -1;
        int minDist = 9999;

        for (int d = 0; d < 4; ++d) {
            int nx = cellX + dx[d];
            int ny = cellY + dy[d];

            if (nx < 0 || nx >= MAZE_CELLS || ny < 0 || ny >= MAZE_CELLS) continue;

            bool isBlocked = false;
            if (d == NORTH && knownMaze[cellX][cellY].north) isBlocked = true;
            if (d == EAST  && knownMaze[cellX][cellY].east)  isBlocked = true;
            if (d == SOUTH && knownMaze[cellX][cellY].south) isBlocked = true;
            if (d == WEST  && knownMaze[cellX][cellY].west)  isBlocked = true;

            if (!isBlocked && floodDist[nx][ny] < minDist) {
                minDist = floodDist[nx][ny];
                bestDir = d;
            }
        }

        if (bestDir == -1) return;

        int turnOffset = (bestDir - (int)orientation + 4) % 4;

        if (turnOffset == 0) {
            commandQueue.push(CMD_MOVE_FORWARD_ONE_CELL);
            physicalMotion.nextCellX = cellX + dx[bestDir];
            physicalMotion.nextCellY = cellY + dy[bestDir];
            physicalMotion.nextOrientation = orientation;
        } 
        else if (turnOffset == 1) {
            commandQueue.push(CMD_TURN_RIGHT);
            physicalMotion.nextCellX = cellX;
            physicalMotion.nextCellY = cellY;
            physicalMotion.nextOrientation = (Direction)((orientation + 1) % 4);
        } 
        else if (turnOffset == 3) {
            commandQueue.push(CMD_TURN_LEFT);
            physicalMotion.nextCellX = cellX;
            physicalMotion.nextCellY = cellY;
            physicalMotion.nextOrientation = (Direction)((orientation + 3) % 4);
        } 
        else if (turnOffset == 2) {
            commandQueue.push(CMD_TURN_180);
            physicalMotion.nextCellX = cellX;
            physicalMotion.nextCellY = cellY;
            physicalMotion.nextOrientation = (Direction)((orientation + 2) % 4);
        }
    }

    double normalizeAngle(double angle) const {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void updatePhysicalController(double dt) {
        if (!pidEnabled || isCrashed) return;

        if (!physicalMotion.active) {
            if (commandQueue.empty()) {
                runFloodfillBrain();
                if (commandQueue.empty()) {
                    v = 0.0; w = 0.0;
                    return;
                }
            }

            physicalMotion.currentCmd = commandQueue.front();
            commandQueue.pop();
            physicalMotion.active = true;
            physicalMotion.startX = x;
            physicalMotion.startY = y;

            physicalMotion.targetAngle = getCardinalAngle(physicalMotion.nextOrientation);

            errorSum = 0.0;
            lastError = 0.0;
        }

        if (physicalMotion.currentCmd == CMD_MOVE_FORWARD_ONE_CELL) {
            v = 150.0;

            float leftDist  = sensors[1].distance;
            float rightDist = sensors[3].distance;

            float idealClearance = 44.0f;
            float wallDetectRange = 100.0f;

            bool hasLeft  = leftDist < wallDetectRange;
            bool hasRight = rightDist < wallDetectRange;

            double rawError = 0.0;

            if (hasLeft && hasRight) {
                rawError = rightDist - leftDist;
            } else if (hasLeft) {
                rawError = (idealClearance - leftDist) * 2.0;
            } else if (hasRight) {
                rawError = (rightDist - idealClearance) * 2.0;
            }

            // Error normalization [-1.0, 1.0]
            double error = std::clamp(rawError / 40.0, -1.0, 1.0);

            errorSum += error * dt;
            errorSum = std::clamp(errorSum, -2.0, 2.0);

            double errorDeriv = (dt > 0.0001) ? (error - lastError) / dt : 0.0;
            lastError = error;

            // Inverted sign for Raylib down-directed Y axis
            double outputW = -(Kp * error + Ki * errorSum + Kd * errorDeriv);
            w = std::clamp(outputW, -2.0, 2.0);

            double distTraveled = std::hypot(x - physicalMotion.startX, y - physicalMotion.startY);

            if (distTraveled >= CELL_PITCH_MM || sensors[2].distance < 42.0f) {
                v = 0.0;
                w = 0.0;

                cellX = physicalMotion.nextCellX;
                cellY = physicalMotion.nextCellY;
                orientation = physicalMotion.nextOrientation;

                // HARD SNAP physical position and angle to grid cell center
                x = (cellX * CELL_PITCH_MM) + (CELL_PITCH_MM / 2.0);
                y = ((MAZE_CELLS - 1 - cellY) * CELL_PITCH_MM) + (CELL_PITCH_MM / 2.0);
                theta = getCardinalAngle(orientation);

                physicalMotion.active = false;
            }
        } 
        else {
            v = 0.0;
            double angleDiff = normalizeAngle(physicalMotion.targetAngle - theta);

            if (std::abs(angleDiff) < 0.02) {
                theta = physicalMotion.targetAngle;
                w = 0.0;

                cellX = physicalMotion.nextCellX;
                cellY = physicalMotion.nextCellY;
                orientation = physicalMotion.nextOrientation;

                // HARD SNAP physical position to grid cell center after turn
                x = (cellX * CELL_PITCH_MM) + (CELL_PITCH_MM / 2.0);
                y = ((MAZE_CELLS - 1 - cellY) * CELL_PITCH_MM) + (CELL_PITCH_MM / 2.0);

                physicalMotion.active = false;
            } else {
                double turnSpeed = angleDiff * 6.0;
                turnSpeed = std::clamp(turnSpeed, -3.0, 3.0);
                if (std::abs(turnSpeed) < 0.6) turnSpeed = (turnSpeed > 0) ? 0.6 : -0.6;
                w = turnSpeed;
            }
        }
    }

    void updateSensors(const Maze& maze) {
        float cosT = cos(theta);
        float sinT = sin(theta);

        for (auto& s : sensors) {
            double rayAngle = theta + s.angleOffset;

            Vector2 rayStart = { 
                (float)(x + (s.localMount.x * cosT - s.localMount.y * sinT)), 
                (float)(y + (s.localMount.x * sinT + s.localMount.y * cosT)) 
            };

            Vector2 rayEnd = { 
                (float)(rayStart.x + s.maxRange * cos(rayAngle)), 
                (float)(rayStart.y + s.maxRange * sin(rayAngle)) 
            };

            float minDistance = s.maxRange;
            Vector2 closestHit = rayEnd;

            for (const auto& w : maze.walls) {
                if (CheckCollisionPointRec(rayStart, w.rect)) {
                    minDistance = 0.0f;
                    closestHit = rayStart;
                    break;
                }

                Vector2 rTL = { w.rect.x, w.rect.y };
                Vector2 rTR = { w.rect.x + w.rect.width, w.rect.y };
                Vector2 rBL = { w.rect.x, w.rect.y + w.rect.height };
                Vector2 rBR = { w.rect.x + w.rect.width, w.rect.y + w.rect.height };

                Vector2 edges[4][2] = { {rTL, rTR}, {rTR, rBR}, {rBR, rBL}, {rBL, rTL} };

                for (int i = 0; i < 4; ++i) {
                    Vector2 hit;
                    if (CheckCollisionLines(rayStart, rayEnd, edges[i][0], edges[i][1], &hit)) {
                        float dist = std::hypot(rayStart.x - hit.x, rayStart.y - hit.y);
                        if (dist < minDistance) {
                            minDistance = dist;
                            closestHit = hit;
                        }
                    }
                }
            }

            s.distance = minDistance;
            s.hitPoint = closestHit;
        }
    }

    std::vector<Vector2> getCorners() const {
        float halfL = length / 2.0f;
        float halfW = width / 2.0f;

        Vector2 localCorners[4] = {
            {  halfL,  halfW },
            {  halfL, -halfW },
            { -halfL, -halfW },
            { -halfL,  halfW }
        };

        std::vector<Vector2> worldCorners(4);
        float cosT = cos(theta);
        float sinT = sin(theta);

        for (int i = 0; i < 4; ++i) {
            worldCorners[i].x = (float)(x + (localCorners[i].x * cosT - localCorners[i].y * sinT));
            worldCorners[i].y = (float)(y + (localCorners[i].x * sinT + localCorners[i].y * cosT));
        }

        return worldCorners;
    }

    bool checkLineRectCollision(Vector2 p1, Vector2 p2, Rectangle rec) const {
        Vector2 rTL = { rec.x, rec.y };
        Vector2 rTR = { rec.x + rec.width, rec.y };
        Vector2 rBL = { rec.x, rec.y + rec.height };
        Vector2 rBR = { rec.x + rec.width, rec.y + rec.height };

        if (CheckCollisionPointRec(p1, rec) || CheckCollisionPointRec(p2, rec)) return true;
        if (CheckCollisionLines(p1, p2, rTL, rTR, nullptr)) return true;
        if (CheckCollisionLines(p1, p2, rTR, rBR, nullptr)) return true;
        if (CheckCollisionLines(p1, p2, rBR, rBL, nullptr)) return true;
        if (CheckCollisionLines(p1, p2, rBL, rTL, nullptr)) return true;

        return false;
    }

    bool checkWallCollision(const Maze& maze) const {
        std::vector<Vector2> corners = getCorners();

        for (int i = 0; i < 4; ++i) {
            Vector2 p1 = corners[i];
            Vector2 p2 = corners[(i + 1) % 4];

            for (const auto& w : maze.walls) {
                if (checkLineRectCollision(p1, p2, w.rect)) return true;
            }
        }
        return false;
    }

    void update(double dt, const Maze& maze) {
        if (isCrashed) {
            v = 0.0; w = 0.0;
            return;
        }

        updateSensors(maze);

        if (pidEnabled) {
            updatePhysicalController(dt);
        }

        x += v * cos(theta) * dt;
        y += v * sin(theta) * dt;
        theta += w * dt;

        if (checkWallCollision(maze)) {
            isCrashed = true;
            v = 0.0; w = 0.0;
        }
    }

    void draw(float scale) const {
        float cosT = cos(theta);
        float sinT = sin(theta);

        for (const auto& s : sensors) {
            Vector2 rayStart = { 
                (float)((x + (s.localMount.x * cosT - s.localMount.y * sinT)) * scale), 
                (float)((y + (s.localMount.x * sinT + s.localMount.y * cosT)) * scale) 
            };
            Vector2 hitPos = { s.hitPoint.x * scale, s.hitPoint.y * scale };

            DrawLineV(rayStart, hitPos, RED);
            DrawCircleV(hitPos, 2.0f, RED);
            DrawCircleV(rayStart, 2.5f, YELLOW);
        }

        Rectangle rect = { 0, 0, length * scale, width * scale };
        Vector2 origin = { (length * scale) / 2.0f, (width * scale) / 2.0f };
        float rotationDegrees = (float)(theta * 180.0 / M_PI);

        DrawRectanglePro(
            { (float)(x * scale), (float)(y * scale), rect.width, rect.height },
            origin,
            rotationDegrees,
            isCrashed ? RED : BLUE
        );

        Vector2 pos = { (float)(x * scale), (float)(y * scale) };
        Vector2 headingEnd = {
            (float)((x + (length / 2.0f) * cos(theta)) * scale),
            (float)((y + (length / 2.0f) * sin(theta)) * scale)
        };
        DrawLineEx(pos, headingEnd, 2.0f, WHITE);
    }
};

#endif