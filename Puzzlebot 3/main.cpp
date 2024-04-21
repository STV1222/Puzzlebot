#include "mbed.h"
#include "hcsr04.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//PC10 TRIGGER PC11 ECHO 
HCSR04 sensorfront(PC_10, PC_11); 
//PA6 TRIGGER PA7 ECHO 
HCSR04 sensorback(D12, D11); 
//PA4 TRIGGER PA5 ECHO 
HCSR04 sensorleft(A2, D13); 
//PB0 TRIGGER PB1 ECHO 
HCSR04 sensorright(PB_0, PB_1); 

InterruptIn button(D4);
RawSerial myRawSerial(PA_9,PA_10, 9600); //Creates RawSerial object using USART6

#define MAX_ROWS 40
#define MAX_COLS 40
#define TOTAL_NODES (MAX_ROWS * MAX_COLS)
#define LARGE_VALUE 9999
#define MAX_TURNS 100  // Assuming a maximum of 100 turns, adjust based on expected path complexity

int openList[TOTAL_NODES] = {0};
int closedList[TOTAL_NODES] = {0};
double g[TOTAL_NODES];
double h[TOTAL_NODES];
double f[TOTAL_NODES];
int parents[TOTAL_NODES];
int turnPositions[MAX_TURNS];
int turnCount = 0;
int move = 1;
int stop = 0;
int turnright = 2;
int turnleft = 3;


#define MAX_OBSTACLES 20

Serial pc(USBTX, NC);

typedef enum {
    NORTH, SOUTH, EAST, WEST, NORTH_EAST, NORTH_WEST, SOUTH_EAST, SOUTH_WEST, UNKNOWN
} Direction;

typedef struct {
    int orig_x1, orig_y1; // Original top-left corner
    int orig_x2, orig_y2; // Original bottom-right corner
    int x1, y1; // Top-left corner
    int x2, y2; // Bottom-right corner
} Obstacle;

typedef struct {
    int north;
    int south;
    int east;
    int west;
} DistanceToObstacle;

DistanceToObstacle distances[TOTAL_NODES];  // Array to hold distance data for all nodes

typedef struct {
    int index;  // Node index of the turn
    Direction fromDirection;  // Direction from which the turn is made
    Direction toDirection;    // Direction to which the turn is made
} TurnInfo;

TurnInfo turnDetails[MAX_TURNS];  // Array to store details about turns

const char* directionToString(Direction dir) { //for the use of printing string 
    switch(dir) {
        case NORTH: return "North";
        case SOUTH: return "South";
        case EAST: return "East";
        case WEST: return "West";
        case NORTH_EAST: return "North-East";
        case NORTH_WEST: return "North-West";
        case SOUTH_EAST: return "South-East";
        case SOUTH_WEST: return "South-West";
        default: return "Unknown";
    }
}
Obstacle obstacles[MAX_OBSTACLES];
int numObstacles = 0;


// Example initialization (could be done in a setup function)
void initObstacles() { //initialize known obstacles
    obstacles[0] = (Obstacle){.orig_x1 = 5, .orig_y1 = 5, .orig_x2 = 11, .orig_y2 = 7}; // First obstacle
    obstacles[1] = (Obstacle){.orig_x1 = 12, .orig_y1 = 15, .orig_x2 = 15, .orig_y2 = 25}; // Second obstacle
    obstacles[2] = (Obstacle){.orig_x1 = 16, .orig_y1 = 8, .orig_x2 = 17, .orig_y2 = 20}; // Second obstacle
    obstacles[3] = (Obstacle){.orig_x1 = 17, .orig_y1 = 1, .orig_x2 = 23, .orig_y2 = 20}; // Second obstacle
    obstacles[4] = (Obstacle){.orig_x1 = 7, .orig_y1 = 30, .orig_x2 = 17, .orig_y2 = 32}; // Second obstacle
    obstacles[5] = (Obstacle){.orig_x1 = 23, .orig_y1 = 27, .orig_x2 = 25, .orig_y2 = 40}; // Second obstacle
    obstacles[6] = (Obstacle){.orig_x1 = 30, .orig_y1 = 20, .orig_x2 = 34, .orig_y2 = 35}; // Second obstacle
    numObstacles = 7; // Update this as you add more obstacles
}

void expandObstacles() { //considering the size of the robot to be a maximum radius of 10 cm 
    for (int i = 0; i < numObstacles; i++) {
        // Copy original dimensions
        obstacles[i].x1 = obstacles[i].orig_x1;
        obstacles[i].y1 = obstacles[i].orig_y1;
        obstacles[i].x2 = obstacles[i].orig_x2;
        obstacles[i].y2 = obstacles[i].orig_y2;
        // Expand each obstacle by 2 nodes in each direction
        // Make sure not to exceed the boundaries of your environment
        obstacles[i].x1 = (obstacles[i].x1 - 2 >= 1) ? obstacles[i].x1 - 2 : 1;
        obstacles[i].y1 = (obstacles[i].y1 - 2 >= 1) ? obstacles[i].y1 - 2 : 1;
        obstacles[i].x2 = (obstacles[i].x2 + 2 <= MAX_COLS) ? obstacles[i].x2 + 2 : MAX_COLS;
        obstacles[i].y2 = (obstacles[i].y2 + 2 <= MAX_ROWS) ? obstacles[i].y2 + 2 : MAX_ROWS;
    }
}

void printObstacleDimensions() {
    for (int i = 0; i < numObstacles; i++) {
        printf("\n\rObstacle %d: (%d, %d) to (%d, %d)\n", i+1, 
               obstacles[i].x1, obstacles[i].y1, 
               obstacles[i].x2, obstacles[i].y2);
    }
}

// Custom max and min functions
int customMax(int a, int b) {
    return (a > b) ? a : b;
}

int customMin(int a, int b) {
    return (a < b) ? a : b;
}

void addObstacleAhead(int currentX, int currentY, const char* direction) { // add new obstacle from ultrasonic sensor and expand the obstacle avoidance area
    if (numObstacles >= MAX_OBSTACLES) {
        printf("\rMaximum number of obstacles reached. Cannot add more.\n");
        return;
    }

    int newX = currentX, newY = currentY;

    // Adjust the obstacle position based on the direction
    if (strcmp(direction, "North") == 0) newY += 1;
    else if (strcmp(direction, "South") == 0) newY -= 1;
    else if (strcmp(direction, "East") == 0) newX += 1;
    else if (strcmp(direction, "West") == 0) newX -= 1;
    else if (strcmp(direction, "North-East") == 0) {
        newX += 1;
        newY += 1;
    }
    else if (strcmp(direction, "North-West") == 0) {
        newX -= 1;
        newY += 1;
    }
    else if (strcmp(direction, "South-East") == 0) {
        newX += 1;
        newY -= 1;
    }
    else if (strcmp(direction, "South-West") == 0) {
        newX -= 1;
        newY -= 1;
    }

    obstacles[numObstacles].x1 = newX;
    obstacles[numObstacles].y1 = newY;
    obstacles[numObstacles].x2 = newX;
    obstacles[numObstacles].y2 = newY;

    // Expand obstacle except on the robot's current side
    if (strcmp(direction, "North") == 0) {
        obstacles[numObstacles].y2 += 2;
        obstacles[numObstacles].x1 -= 2;
        obstacles[numObstacles].x2 += 2;
    } 
    else if (strcmp(direction, "South") == 0) {
        obstacles[numObstacles].y1 -= 2;
        obstacles[numObstacles].x1 -= 2;
        obstacles[numObstacles].x2 += 2;
    } 
    else if (strcmp(direction, "East") == 0) {
        obstacles[numObstacles].x2 += 2;
        obstacles[numObstacles].y1 -= 2;
        obstacles[numObstacles].y2 += 2;
    } 
    else if (strcmp(direction, "West") == 0) {
        obstacles[numObstacles].x1 -= 2;
        obstacles[numObstacles].y1 -= 2;
        obstacles[numObstacles].y2 += 2;
    }
    else if (strcmp(direction, "North-East") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY-1, .x2 = newX+1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY, .x2 = newX+1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+2, .y1 = newY, .x2 = newX+2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY+1, .x2 = newX+1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+1, .x2 = newX, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY+1, .x2 = newX-1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+2, .x2 = newX, .y2 = newY+2};
    }
    else if (strcmp(direction, "North-West") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY, .x2 = newX-1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-2, .y1 = newY, .x2 = newX-2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY-1, .x2 = newX-1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY+1, .x2 = newX-1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+1, .x2 = newX, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY+1, .x2 = newX+1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+2, .x2 = newX, .y2 = newY+2};
    }
    else if (strcmp(direction, "South-East") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY, .x2 = newX+1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+2, .y1 = newY, .x2 = newX+2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY-1, .x2 = newX-1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY-1, .x2 = newX+1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-1, .x2 = newX, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY+1, .x2 = newX+1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-2, .x2 = newX, .y2 = newY-2};
    }
    else if (strcmp(direction, "South-West") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY, .x2 = newX-1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-2, .y1 = newY, .x2 = newX-2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY+1, .x2 = newX-1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY-1, .x2 = newX-1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-1, .x2 = newX, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY-1, .x2 = newX+1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-2, .x2 = newX, .y2 = newY-2};
    }
    
    // Make sure the obstacle stays within bounds
    obstacles[numObstacles].x1 = customMax(1, obstacles[numObstacles].x1);
    obstacles[numObstacles].y1 = customMax(1, obstacles[numObstacles].y1);
    obstacles[numObstacles].x2 = customMin(MAX_COLS, obstacles[numObstacles].x2);
    obstacles[numObstacles].y2 = customMin(MAX_ROWS, obstacles[numObstacles].y2);

    numObstacles++;
    printObstacleDimensions();
    // printf("\rNew obstacle added and expanded at position (%d, %d) to (%d, %d).\n",
    //     obstacles[numObstacles - 1].x1, obstacles[numObstacles - 1].y1, 
    //     obstacles[numObstacles - 1].x2, obstacles[numObstacles - 1].y2);
}

volatile bool trigger_flag = false;
volatile int globalCurrentNodeX = 0, globalCurrentNodeY =0; // current running position
volatile int previousNodeX = -1, previousNodeY = -1; // Previous position
char* currentFacingDirection;
int start_x = 40; //set goal positions
int start_y = 40;
int goal_x = 1; //set start positions
int goal_y = 1;

int xyToIndex(int x, int y) { return (y - 1) * MAX_COLS + (x - 1); }
void indexToXY(int index, int *x, int *y) {
    *x = index % MAX_COLS + 1;
    *y = index / MAX_COLS + 1;
}

int goalIdx = xyToIndex(goal_x, goal_y);
int idx = goalIdx;
int parentIdx;

void initializeGrid() {
    for (int i = 0; i < TOTAL_NODES; i++) {
        g[i] = LARGE_VALUE;
        h[i] = LARGE_VALUE;
        f[i] = LARGE_VALUE;
        parents[i] = -1;
        openList[i] = 0;
        closedList[i] = 0;
    }
}

int isValidPosition(int x, int y) {

    if (x < 1 || x > MAX_COLS || y < 1 || y > MAX_ROWS) return 0; // Check bounds first

    for (int i = 0; i < numObstacles; i++) {
        // Check if position is inside the current obstacle
        if (x >= obstacles[i].x1 && x <= obstacles[i].x2 && y >= obstacles[i].y1 && y <= obstacles[i].y2) {
            return 0; // Position is invalid if it's inside any obstacle
        }
    }
    return 1; // Position is valid if it's outside all obstacles
}

int isOriginalValidPosition(int x, int y) { //use to check the orginal size of the obstacle
    for (int i = 0; i < numObstacles; i++) {
        if (x >= obstacles[i].orig_x1 && x <= obstacles[i].orig_x2 && y >= obstacles[i].orig_y1 && y <= obstacles[i].orig_y2) {
            return 0;  // Position is invalid (inside an original dimension obstacle)
        }
    }
    return 1;  // Position is valid (outside all original dimension obstacles)
}

Direction getDirection(int fromX, int fromY, int toX, int toY) {
    if (toY > fromY) {
        if (toX > fromX) return NORTH_EAST;
        else if (toX < fromX) return NORTH_WEST;
        else return NORTH;
    }
    else if (toY < fromY) {
        if (toX > fromX) return SOUTH_EAST;
        else if (toX < fromX) return SOUTH_WEST;
        else return SOUTH;
    } 
    else {
        if (toX > fromX) return EAST;
        else if (toX < fromX) return WEST;
    }
    return UNKNOWN;
}

double calculateDistance(int x1, int y1, int x2, int y2) {
    return sqrt(pow((double)abs(x1 - x2), 2) + pow((double)abs(y1 - y2), 2));
}

void printPathAndDistance(int goalIdx) {
    int idx = goalIdx;
    double pathDistance = 0;
    int steps = 0;
    int parentIdx;

    printf("\rPath Generated:\n\r");
    while (parents[idx] != -1) {
        parentIdx = parents[idx];
        pathDistance += calculateDistance(idx % MAX_COLS + 1, idx / MAX_COLS + 1, parentIdx % MAX_COLS + 1, parentIdx / MAX_COLS + 1);
        printf("(%d, %d) -> ", idx % MAX_COLS + 1, idx / MAX_COLS + 1);
        idx = parentIdx;
        steps++;
        if (steps % 5 == 0) printf("\n\r"); // Break line for readability
    }
    printf("(%d, %d)\n", idx % MAX_COLS + 1, idx / MAX_COLS + 1); // Print start node
    
    printf("\rTotal path distance: %.2f units\n", pathDistance);
    wait(12);
}

void printpath(int startX, int startY, int goalX, int goalY){    // Not using now, but can be use for testing 
    int goalIdx = xyToIndex(goalX, goalY);
    int idx = goalIdx;
    int parentIdx;
    while (parents[idx] != -1) {

        parentIdx = parents[idx];
        printf("\r(%d, %d)\n", idx % MAX_COLS + 1, idx / MAX_COLS + 1);
        wait(1);
        idx = parentIdx;
    }
    printf("\r(%d, %d)\n", idx % MAX_COLS + 1, idx / MAX_COLS + 1); // Print start node
}

void analyzePathAndRecordTurns() {
    if (goalIdx == -1) {
        printf("\rNo path has been computed yet.\n");
        return;
    }

    int currentIdx = goalIdx;
    int parentIdx = parents[currentIdx];
    if (parentIdx == -1) {
        printf("\rPath is not available or too short to determine turns.\n");
        return;
    }

    int currentX, currentY, nextX, nextY;
    indexToXY(currentIdx, &currentX, &currentY);

    Direction currentDirection = UNKNOWN;
    Direction nextDirection;

    // Initialize the first direction
    indexToXY(parentIdx, &nextX, &nextY);
    currentDirection = getDirection(currentX, currentY, nextX, nextY);

    printf("\rTurn positions and directions:\n");

    while (parents[parentIdx] != -1) {
        currentIdx = parentIdx;
        parentIdx = parents[currentIdx];

        currentX = nextX;
        currentY = nextY;
        indexToXY(parentIdx, &nextX, &nextY);

        nextDirection = getDirection(currentX, currentY, nextX, nextY);

        if (nextDirection != currentDirection) {
            if (turnCount < MAX_TURNS) { // Check to avoid overflow
                turnDetails[turnCount].index = xyToIndex(currentX, currentY);
                turnDetails[turnCount].fromDirection = currentDirection;
                turnDetails[turnCount].toDirection = nextDirection;
                // printf("\rTurn at (%d, %d) from %s to %s\n", currentX, currentY, directionToString(currentDirection), directionToString(nextDirection));
                turnCount++;
            }
            currentDirection = nextDirection;
        }
    }

    printf("\rTotal number of turns: %d\n", turnCount);
}

int distanceToObstacleInDirection(int x, int y, Direction direction) {
    int distance = -1; // Default to -1 if no conditions are met (should not happen with boundary checks)

    switch (direction) {
        case NORTH:
            distance = MAX_ROWS - y; // Maximum distance to the north boundary
            for (int checkY = y + 1; checkY <= MAX_ROWS; checkY++) {
                if (!isOriginalValidPosition(x, checkY)) {
                    distance = checkY - y - 1; // Subtract one to account for zero-based offset
                    break;
                }
            }
            break;
        case SOUTH:
            distance = y; // Maximum distance to the south boundary
            for (int checkY = y - 1; checkY >= 1; checkY--) {
                if (!isOriginalValidPosition(x, checkY)) {
                    distance = y - checkY; // Subtract one to account for zero-based offset
                    break;
                }
            }
            break;
        case EAST:
            distance = MAX_COLS - x; // Maximum distance to the east boundary
            for (int checkX = x + 1; checkX <= MAX_COLS; checkX++) {
                if (!isOriginalValidPosition(checkX, y)) {
                    distance = checkX - x; // Subtract one to account for zero-based offset
                    break;
                }
            }
            break;
        case WEST:
            distance = x; // Maximum distance to the west boundary
            for (int checkX = x - 1; checkX >= 1; checkX--) {
                if (!isOriginalValidPosition(checkX, y)) {
                    distance = x - checkX - 1; // Subtract one to account for zero-based offset
                    break;
                }
            }
            break;
        case NORTH_EAST:
            distance = customMin(MAX_ROWS - y, MAX_COLS - x); // Maximum diagonal distance to the northeast boundary
            for (int step = 1; x + step <= MAX_COLS && y + step <= MAX_ROWS; step++) {
                if (!isOriginalValidPosition(x + step, y + step)) {
                    distance = step; // Subtract one to get actual obstacle distance
                    break;
                }
            }
            break;
        case NORTH_WEST:
            distance = customMin(MAX_ROWS - y, x - 1); // Maximum diagonal distance to the northwest boundary
            for (int step = 1; x - step >= 1 && y + step <= MAX_ROWS; step++) {
                if (!isOriginalValidPosition(x - step, y + step)) {
                    distance = step - 1; // Subtract one to get actual obstacle distance
                    break;
                }
            }
            break;
        case SOUTH_EAST:
            distance = customMin(y, MAX_COLS - x); // Maximum diagonal distance to the southeast boundary
            for (int step = 1; x + step <= MAX_COLS && y - step >= 1; step++) {
                if (!isOriginalValidPosition(x + step, y - step)) {
                    distance = step; // Subtract one to get actual obstacle distance
                    break;
                }
            }
            break;
        case SOUTH_WEST:
            distance = customMin(y, x); // Maximum diagonal distance to the southwest boundary
            for (int step = 1; x - step >= 1 && y - step >= 1; step++) {
                if (!isOriginalValidPosition(x - step, y - step)) {
                    distance = step - 1; // Subtract one to get actual obstacle distance
                    break;
                }
            }
            break;
        default:
            break;
    }

    return distance;
}

void computeAndStoreDistances(int idx, int x, int y) { //Function to Compute and Store Distances
    distances[idx].north = distanceToObstacleInDirection(x, y, NORTH);
    distances[idx].south = distanceToObstacleInDirection(x, y, SOUTH);
    distances[idx].east = distanceToObstacleInDirection(x, y, EAST);
    distances[idx].west = distanceToObstacleInDirection(x, y, WEST);
}

int printDistanceDataForNode(int idx, Direction dir) { //Function to Retrieve and Print Distance Data
    int x, y;
    indexToXY(idx, &x, &y);
    printf("\rPosition (%d, %d) - North: %d, South: %d, East: %d, West: %d\n",
           x, y, distances[idx].north, distances[idx].south, distances[idx].east, distances[idx].west);
    wait(5);
    switch (dir) {
        case NORTH:
            return distances[idx].north;
        case SOUTH:
            return distances[idx].south;
        case EAST:
            return distances[idx].east;
        case WEST:
            return distances[idx].west;
        default:
            // Optionally handle an undefined direction
            printf("Invalid direction\n");
        return -1; // Return an error code or invalid distance
    }

}  

// Function to find the most likely current position based on sensor data
void locateCurrentPosition(float frontDist, float backDist, float rightDist, float leftDist, float tolerance) {
    double minError = 9999999.0; // Large initial value for comparison
    int bestX = -1;
    int bestY = -1;
    int matchingSensors;

    for (int x = 1; x <= MAX_COLS; x++) {
        for (int y = 1; y <= MAX_ROWS; y++) {
            int idx = xyToIndex(x, y);
            matchingSensors = 0; // Reset count for each cell

            // Check each sensor's error and count how many are within tolerance
            if (fabs(distances[idx].north * 5 - frontDist) <= tolerance) matchingSensors++;
            if (fabs(distances[idx].south * 5 - backDist) <= tolerance) matchingSensors++;
            if (fabs(distances[idx].east * 5 - rightDist) <= tolerance) matchingSensors++;
            if (fabs(distances[idx].west * 5 - leftDist) <= tolerance) matchingSensors++;

            // Only consider positions where at least three sensors are within the tolerance
            if (matchingSensors >= 3) {
                double error = 0;
                error += fabs(distances[idx].north * 5 - frontDist); // Multiply by 5 for unit conversion
                error += fabs(distances[idx].south * 5 - backDist);
                error += fabs(distances[idx].east * 5 - rightDist);
                error += fabs(distances[idx].west * 5 - leftDist);

                // Update the best position if this one has a lower error
                if (error < minError) {
                    minError = error;
                    bestX = x;
                    bestY = y;
                }
            }
        }
    }

    if (bestX != -1) {
        printf("\rMost likely position based on sensor data: (%d, %d) with error: %f\n", bestX, bestY, minError);
    } else {
        printf("\rNo position closely matches the sensor data within the tolerance or enough sensors matching.\n");
    }
}

void tangentBugAvoidance(float distancefront, float distanceback, float distanceright, float distanceleft, float goalDirection) {
    // Assuming goalDirection is an angle (0 to 360 degrees) relative to the current robot heading
    
    float leftClearance = distanceleft;
    float rightClearance = distanceright;
    float backwardClearance = distanceback;
    float recordturn = 0;
    while(1){
        distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
        distanceright = sensorright.distance(); // measures the distance from HC-SR04
        printf("\rSensor left: %f\n", distanceleft);
        printf("\rSensor right: %f\n", distanceright);

        if(distanceright>30){
            myRawSerial.putc(static_cast<char>(turnright));
            myRawSerial.putc('\n');  // Turn right
            wait(1);
            myRawSerial.putc(static_cast<char>(stop));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            wait(1);
            myRawSerial.putc(static_cast<char>(move));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            wait(0.4);
            myRawSerial.putc(static_cast<char>(stop));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            wait(1);
            while(1){
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distancefront = sensorfront.distance();
                if(distanceleft>18){
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnleft));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.4);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                    recordturn+=0.3;
                    if (recordturn>=1.5) {
                        myRawSerial.putc(static_cast<char>(move));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(1.5);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(1);
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.6);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        return;
                    }
                }
                else if(distancefront<15){
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    break;
                }
                else {
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
            }
        }
        else if(distanceleft>30){
            myRawSerial.putc(static_cast<char>(turnleft));
            myRawSerial.putc('\n');  // Turn right
            wait(1);
            myRawSerial.putc(static_cast<char>(stop));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            wait(1);
            myRawSerial.putc(static_cast<char>(move));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            wait(0.4);
            myRawSerial.putc(static_cast<char>(stop));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            wait(1);
            while(1){
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distancefront = sensorfront.distance();
                if(distanceright>18){
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.4);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                    recordturn+=0.3;
                    if (recordturn>=1.5) {
                        myRawSerial.putc(static_cast<char>(move));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(1.5);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(1);
                        myRawSerial.putc(static_cast<char>(turnleft));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.6);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        return;
                    }
                }
                else if(distancefront<15){
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    break;
                }
                else {
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
            }
        }
        else {
            printf("\rNo path found\n");
            wait(100);
        }
    }
    wait(1);  // Add delay to allow the robot to perform the maneuver
    // Continue with the new heading or reassess the obstacle situation
}

void printTurnDetails() {
    for (int i = 0; i < turnCount; i++) {
        int x, y;
        indexToXY(turnDetails[i].index, &x, &y);
        const char* currentDirection = directionToString(turnDetails[i].fromDirection);
        const char* nextDirection = directionToString(turnDetails[i].toDirection); 

        printf("\rNext turn at position (%d, %d):\n", x, y); // Prints the current position
        wait(10);

        if (strcmp(currentDirection, "North") == 0) {
            printf("\rCurrent Direction: North\n");
            printf("\rNext Direction: %s\n", nextDirection);
            int nodeIndex = xyToIndex(x, y);
            int expectedDistance_north = printDistanceDataForNode(nodeIndex, NORTH); // Get the distance once and use it in your logic
            int North_Distance = expectedDistance_north*5; //since the actual environment is 200*200
            int expectedDistance_east = printDistanceDataForNode(nodeIndex, EAST); // Get the distance once and use it in your logic
            int East_Distance = expectedDistance_east*5; //since the actual environment is 200*200
            int expectedDistance_south = printDistanceDataForNode(nodeIndex, SOUTH); // Get the distance once and use it in your logic
            int South_Distance = expectedDistance_south*5; //since the actual environment is 200*200
            int expectedDistance_west = printDistanceDataForNode(nodeIndex, WEST); // Get the distance once and use it in your logic
            int West_Distance = expectedDistance_west*5; //since the actual environment is 200*200

            float distancefront, distanceback, distanceright, distanceleft;
            do {
                distancefront = sensorfront.distance(); // measures the distance from HC-SR04
                distanceback = sensorback.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                printf("\n\rCurrent sensorfront distance: %f cm\n", distancefront);
                printf("\rCurrent sensorback distance: %f cm\n", distanceback);
                printf("\rCurrent sensorright distance: %f cm\n", distanceright);
                printf("\rCurrent sensorleft distance: %f cm\n", distanceleft);
                myRawSerial.putc(static_cast<char>(move));
                myRawSerial.putc('\n');  // Send newline character to mark end of message

                if (distancefront >= North_Distance - 2 && distancefront <= North_Distance + 2 || distanceback >= South_Distance - 2 && distanceback <= South_Distance + 2) {
                    printf("REACHES the turn point\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    if (strcmp(nextDirection, "North") == 0) {
                        printf("Next direction is North\n");
                        // Additional code to handle northward movement
                    }
                    else if (strcmp(nextDirection, "South") == 0) {
                        printf("Next direction is South\n");
                        // Additional code to handle southward movement
                    }
                    else if (strcmp(nextDirection, "East") == 0) {
                        printf("Next direction is East\n");
                        // Additional code to handle eastward movement
                    }
                    else if (strcmp(nextDirection, "West") == 0) {
                        printf("Next direction is West\n");
                        // Additional code to handle westward movement
                    }
                    else if (strcmp(nextDirection, "North-East") == 0) {
                        printf("Next direction is North-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for north-eastward movement
                    }
                    else if (strcmp(nextDirection, "North-West") == 0) {
                        printf("Next direction is North-West\n");
                        // Additional code for north-westward movement
                    }
                    else if (strcmp(nextDirection, "South-East") == 0) {
                        printf("Next direction is South-East\n");
                        // Additional code for south-eastward movement
                    }
                    else if (strcmp(nextDirection, "South-West") == 0) {
                        printf("Next direction is South-West\n");
                        // Additional code for south-westward movement
                    }
                    else {
                        printf("Direction is unknown or not specified.\n");
                        // Handle unexpected or unknown direction
                    }
                    break;
                }
                else if (distancefront < 20){ // unknown obstacle detected perform obstacle avoidance
                    printf("\rPerform obstacle avoidance.\n");
                    tangentBugAvoidance(distancefront, distanceback, distanceright, distanceleft, 0);  // Call the tangent bug algorithm
                    distancefront = sensorfront.distance(); // measures the distance from HC-SR04
                    distanceback = sensorback.distance(); // measures the distance from HC-SR04
                    distanceright = sensorright.distance(); // measures the distance from HC-SR04
                    distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                    locateCurrentPosition(distancefront, distanceback, distanceright, distanceleft, 3.0f);  // 5.0f is the tolerance
                    wait(100);
                    // add the new obstacle to map 
                    // use the current sensors distance to obstacles to know what point 
                    // regenerate the path planning algorithm 
                }
                else if (distanceleft < 15){
                    printf("\rLeft too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                else if (distanceright < 15){
                    printf("\right too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                // Additional logic here if needed

            } while (1);

        }
        else if (strcmp(currentDirection, "South") == 0) {
            int nodeIndex = xyToIndex(x, y);
            int expectedDistance_north = printDistanceDataForNode(nodeIndex, NORTH); // Get the distance once and use it in your logic
            int North_Distance = expectedDistance_north*5; //since the actual environment is 200*200
            int expectedDistance_east = printDistanceDataForNode(nodeIndex, EAST); // Get the distance once and use it in your logic
            int East_Distance = expectedDistance_east*5; //since the actual environment is 200*200
            int expectedDistance_south = printDistanceDataForNode(nodeIndex, SOUTH); // Get the distance once and use it in your logic
            int South_Distance = expectedDistance_south*5; //since the actual environment is 200*200
            int expectedDistance_west = printDistanceDataForNode(nodeIndex, WEST); // Get the distance once and use it in your logic
            int West_Distance = expectedDistance_west*5; //since the actual environment is 200*200
            printf("\rCurrent Direction: East\n");
            printf("\rNext Direction: %s\n", nextDirection);
            float distancefront, distanceback, distanceright, distanceleft;
            
            do {
                distancefront = sensorfront.distance(); // measures the distance from HC-SR04
                distanceback = sensorback.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                printf("\n\rCurrent sensorfront distance: %f cm\n", distancefront);
                printf("\rCurrent sensorback distance: %f cm\n", distanceback);
                printf("\rCurrent sensorright distance: %f cm\n", distanceright);
                printf("\rCurrent sensorleft distance: %f cm\n", distanceleft);
                myRawSerial.putc(static_cast<char>(move));
                myRawSerial.putc('\n');  // Send newline character to mark end of message

                if (distancefront >= South_Distance - 2 && distancefront <= South_Distance + 2 || distanceback >= North_Distance - 2 && distanceback <= North_Distance + 2) {
                    printf("REACHES the turn point\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    if (strcmp(nextDirection, "North") == 0) {
                        printf("Next direction is North\n");
                        // Additional code to handle northward movement
                        myRawSerial.putc(static_cast<char>(turnleft));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                    }
                    else if (strcmp(nextDirection, "South") == 0) {
                        printf("Next direction is South\n");
                        // Additional code to handle southward movement
                    }
                    else if (strcmp(nextDirection, "East") == 0) {
                        printf("Next direction is East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code to handle eastward movement
                    }
                    else if (strcmp(nextDirection, "West") == 0) {
                        printf("Next direction is West\n");
                        // Additional code to handle westward movement
                    }
                    else if (strcmp(nextDirection, "North-East") == 0) {
                        printf("Next direction is North-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for north-eastward movement
                    }
                    else if (strcmp(nextDirection, "North-West") == 0) {
                        printf("Next direction is North-West\n");
                        // Additional code for north-westward movement
                    }
                    else if (strcmp(nextDirection, "South-East") == 0) {
                        printf("Next direction is South-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for south-eastward movement
                    }
                    else if (strcmp(nextDirection, "South-West") == 0) {
                        printf("Next direction is South-West\n");
                        // Additional code for south-westward movement
                    }
                    else {
                        printf("Direction is unknown or not specified.\n");
                        // Handle unexpected or unknown direction
                    }
                    break;
                }
                else if (distancefront < 20){ // unknown obstacle detected perform obstacle avoidance
                    printf("\rPerform obstacle avoidance.\n");
                    tangentBugAvoidance(distancefront, distanceback, distanceright, distanceleft, 0);  // Call the tangent bug algorithm
                }
                else if (distanceleft < 15){
                    printf("\rLeft too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                else if (distanceright < 15){
                    printf("\right too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                // Additional logic here if needed
            } while (1);
        }
        else if (strcmp(currentDirection, "East") == 0) {
            int nodeIndex = xyToIndex(x, y);
            int expectedDistance_north = printDistanceDataForNode(nodeIndex, NORTH); // Get the distance once and use it in your logic
            int North_Distance = expectedDistance_north*5; //since the actual environment is 200*200
            int expectedDistance_east = printDistanceDataForNode(nodeIndex, EAST); // Get the distance once and use it in your logic
            int East_Distance = expectedDistance_east*5; //since the actual environment is 200*200
            int expectedDistance_south = printDistanceDataForNode(nodeIndex, SOUTH); // Get the distance once and use it in your logic
            int South_Distance = expectedDistance_south*5; //since the actual environment is 200*200
            int expectedDistance_west = printDistanceDataForNode(nodeIndex, WEST); // Get the distance once and use it in your logic
            int West_Distance = expectedDistance_west*5; //since the actual environment is 200*200
            printf("\rCurrent Direction: East\n");
            printf("\rNext Direction: %s\n", nextDirection);
            float distancefront, distanceback, distanceright, distanceleft;
            
            do {
                distancefront = sensorfront.distance(); // measures the distance from HC-SR04
                distanceback = sensorback.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                printf("\n\rCurrent sensorfront distance: %f cm\n", distancefront);
                printf("\rCurrent sensorback distance: %f cm\n", distanceback);
                printf("\rCurrent sensorright distance: %f cm\n", distanceright);
                printf("\rCurrent sensorleft distance: %f cm\n", distanceleft);
                myRawSerial.putc(static_cast<char>(move));
                myRawSerial.putc('\n');  // Send newline character to mark end of message

                if (distancefront >= East_Distance - 2 && distancefront <= East_Distance + 2 || distanceback >= West_Distance - 2 && distanceback <= West_Distance + 2) {
                    printf("REACHES the turn point\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    if (strcmp(nextDirection, "North") == 0) {
                        printf("Next direction is North\n");
                        // Additional code to handle northward movement
                        myRawSerial.putc(static_cast<char>(turnleft));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                    }
                    else if (strcmp(nextDirection, "South") == 0) {
                        printf("Next direction is South\n");
                        // Additional code to handle southward movement
                    }
                    else if (strcmp(nextDirection, "East") == 0) {
                        printf("Next direction is East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code to handle eastward movement
                    }
                    else if (strcmp(nextDirection, "West") == 0) {
                        printf("Next direction is West\n");
                        // Additional code to handle westward movement
                    }
                    else if (strcmp(nextDirection, "North-East") == 0) {
                        printf("Next direction is North-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for north-eastward movement
                    }
                    else if (strcmp(nextDirection, "North-West") == 0) {
                        printf("Next direction is North-West\n");
                        // Additional code for north-westward movement
                    }
                    else if (strcmp(nextDirection, "South-East") == 0) {
                        printf("Next direction is South-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for south-eastward movement
                    }
                    else if (strcmp(nextDirection, "South-West") == 0) {
                        printf("Next direction is South-West\n");
                        // Additional code for south-westward movement
                    }
                    else {
                        printf("Direction is unknown or not specified.\n");
                        // Handle unexpected or unknown direction
                    }
                    break;
                }
                else if (distancefront < 20){ // unknown obstacle detected perform obstacle avoidance
                    printf("\rPerform obstacle avoidance.\n");
                    tangentBugAvoidance(distancefront, distanceback, distanceright, distanceleft, 0);  // Call the tangent bug algorithm
                }
                else if (distanceleft < 15){
                    printf("\rLeft too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                else if (distanceright < 15){
                    printf("\right too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                // Additional logic here if needed
            } while (1);
        }
        else if (strcmp(currentDirection, "West") == 0) {
            printf("\rWest\n");
        }
        else if (strcmp(currentDirection, "North-East") == 0) {
            int  expectedDistance_northeast = distanceToObstacleInDirection(x, y, NORTH_EAST);
            float NorthEast_Distance = expectedDistance_northeast*sqrt(static_cast<float>(2))*5; // times square root of 2 for diagonal distance and times 5 for real world
            int expectedDistance_southwest = distanceToObstacleInDirection(x, y, SOUTH_WEST);
            float SouthWest_Distance = expectedDistance_southwest*sqrt(static_cast<float>(2))*5; // times square root of 2 for diagonal distance and times 5 for real world
            printf("\rCurrent Direction: North-East\n");
            printf("\rNext Direction: %s\n", nextDirection);
            int nodeIndex = xyToIndex(x, y);
            float distancefront, distanceback, distanceright, distanceleft;
            
            do {
                distancefront = sensorfront.distance(); // measures the distance from HC-SR04
                distanceback = sensorback.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                printf("\n\rCurrent sensorfront distance: %f cm\n", distancefront);
                printf("\rCurrent sensorback distance: %f cm\n", distanceback);
                printf("\rCurrent sensorright distance: %f cm\n", distanceright);
                printf("\rCurrent sensorleft distance: %f cm\n", distanceleft);
                myRawSerial.putc(static_cast<char>(move));
                myRawSerial.putc('\n');  // Send newline character to mark end of message

                if (distancefront >= NorthEast_Distance - 2 && distancefront <= NorthEast_Distance + 2 || distanceback >= SouthWest_Distance - 2 && distanceback <= SouthWest_Distance + 2) {
                    printf("REACHES the turn point\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    if (strcmp(nextDirection, "North") == 0) {
                        printf("Next direction is North\n");
                        // Additional code to handle northward movement
                        myRawSerial.putc(static_cast<char>(turnleft));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                    }
                    else if (strcmp(nextDirection, "South") == 0) {
                        printf("Next direction is South\n");
                        // Additional code to handle southward movement
                    }
                    else if (strcmp(nextDirection, "East") == 0) {
                        printf("Next direction is East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code to handle eastward movement
                    }
                    else if (strcmp(nextDirection, "West") == 0) {
                        printf("Next direction is West\n");
                        // Additional code to handle westward movement
                    }
                    else if (strcmp(nextDirection, "North-East") == 0) {
                        printf("Next direction is North-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for north-eastward movement
                    }
                    else if (strcmp(nextDirection, "North-West") == 0) {
                        printf("Next direction is North-West\n");
                        // Additional code for north-westward movement
                    }
                    else if (strcmp(nextDirection, "South-East") == 0) {
                        printf("Next direction is South-East\n");
                        // Additional code for south-eastward movement
                    }
                    else if (strcmp(nextDirection, "South-West") == 0) {
                        printf("Next direction is South-West\n");
                        // Additional code for south-westward movement
                    }
                    else {
                        printf("Direction is unknown or not specified.\n");
                        // Handle unexpected or unknown direction
                    }
                    break;
                }
                else if (distancefront < 20){ // unknown obstacle detected perform obstacle avoidance
                    printf("\rPerform obstacle avoidance.\n");
                    tangentBugAvoidance(distancefront, distanceback, distanceright, distanceleft, 0);  // Call the tangent bug algorithm
                }
                else if (distanceleft < 15){
                    printf("\rLeft too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                else if (distanceright < 15){
                    printf("\right too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                // Additional logic here if needed
            } while (1);

        }
        else if (strcmp(currentDirection, "South-East") == 0) {
            int  expectedDistance_southeast = distanceToObstacleInDirection(x, y, SOUTH_EAST);
            float SouthEast_Distance = expectedDistance_southeast*sqrt(static_cast<float>(2))*5; // times square root of 2 for diagonal distance and times 5 for real world
            int  expectedDistance_northwest = distanceToObstacleInDirection(x, y, NORTH_WEST);
            float NorthWest_Distance = expectedDistance_northwest*sqrt(static_cast<float>(2))*5; // times square root of 2 for diagonal distance and times 5 for real world

            printf("\rCurrent Direction: North-East\n");
            printf("\rNext Direction: %s\n", nextDirection);
            int nodeIndex = xyToIndex(x, y);
            float distancefront, distanceback, distanceright, distanceleft;
            
            do {
                distancefront = sensorfront.distance(); // measures the distance from HC-SR04
                distanceback = sensorback.distance(); // measures the distance from HC-SR04
                distanceright = sensorright.distance(); // measures the distance from HC-SR04
                distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
                printf("\n\rCurrent sensorfront distance: %f cm\n", distancefront);
                printf("\rCurrent sensorback distance: %f cm\n", distanceback);
                printf("\rCurrent sensorright distance: %f cm\n", distanceright);
                printf("\rCurrent sensorleft distance: %f cm\n", distanceleft);
                myRawSerial.putc(static_cast<char>(move));
                myRawSerial.putc('\n');  // Send newline character to mark end of message

                if (distancefront >= SouthEast_Distance - 2 && distancefront <= SouthEast_Distance + 2 || distanceback >= NorthWest_Distance - 2 && distanceback <= NorthWest_Distance + 2) {
                    printf("REACHES the turn point\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    if (strcmp(nextDirection, "North") == 0) {
                        printf("Next direction is North\n");
                        // Additional code to handle northward movement
                        myRawSerial.putc(static_cast<char>(turnleft));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                    }
                    else if (strcmp(nextDirection, "South") == 0) {
                        printf("Next direction is South\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code to handle southward movement
                    }
                    else if (strcmp(nextDirection, "East") == 0) {
                        printf("Next direction is East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code to handle eastward movement
                    }
                    else if (strcmp(nextDirection, "West") == 0) {
                        printf("Next direction is West\n");
                        // Additional code to handle westward movement
                    }
                    else if (strcmp(nextDirection, "North-East") == 0) {
                        printf("Next direction is North-East\n");
                        myRawSerial.putc(static_cast<char>(turnright));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        wait(0.8);
                        myRawSerial.putc(static_cast<char>(stop));
                        myRawSerial.putc('\n');  // Send newline character to mark end of message
                        // Additional code for north-eastward movement
                    }
                    else if (strcmp(nextDirection, "North-West") == 0) {
                        printf("Next direction is North-West\n");
                        // Additional code for north-westward movement
                    }
                    else if (strcmp(nextDirection, "South-East") == 0) {
                        printf("Next direction is South-East\n");
                        // Additional code for south-eastward movement
                    }
                    else if (strcmp(nextDirection, "South-West") == 0) {
                        printf("Next direction is South-West\n");
                        // Additional code for south-westward movement
                    }
                    else {
                        printf("Direction is unknown or not specified.\n");
                        // Handle unexpected or unknown direction
                    }
                    break;
                }
                else if (distancefront < 20){ // unknown obstacle detected perform obstacle avoidance
                    printf("\rPerform obstacle avoidance.\n");
                    tangentBugAvoidance(distancefront, distanceback, distanceright, distanceleft, 0);  // Call the tangent bug algorithm
                }
                else if (distanceleft < 15){
                    printf("\rLeft too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                else if (distanceright < 15){
                    printf("\right too close, make adjustment.\n");
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(turnright));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.3);
                    myRawSerial.putc(static_cast<char>(stop));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(1);
                    myRawSerial.putc(static_cast<char>(move));
                    myRawSerial.putc('\n');  // Send newline character to mark end of message
                    wait(0.5);
                }
                // Additional logic here if needed
            } while (1);
        }
        else {
            printf("Direction not recognized.\n");
        }
        // printf("\rTurn at (%d, %d) from %s to %s\n",
        //        x, y,
        //        directionToString(turnDetails[i].fromDirection),
        //        directionToString(turnDetails[i].toDirection));
        // printDistanceDataForNode(turnDetails[i].index);
    }
}

void aStarSearch(int startX, int startY, int goalX, int goalY) {
    initializeGrid();
    int startIdx = xyToIndex(startX, startY);
    int goalIdx = xyToIndex(goalX, goalY);
    g[startIdx] = 0;
    h[startIdx] = calculateDistance(startX, startY, goalX, goalY);
    f[startIdx] = h[startIdx];
    openList[startIdx] = 1;

    // Compute initial distances for the start node
    indexToXY(startIdx, &startX, &startY);
    computeAndStoreDistances(startIdx, startX, startY);

    while (1) {
        int currentIdx = -1;
        double minF = LARGE_VALUE;
        for (int i = 0; i < TOTAL_NODES; i++) {
            if (openList[i] && f[i] < minF) {
                minF = f[i];
                currentIdx = i;
            }
        }

        if (currentIdx == -1) {
            printf("\rPath not found.\n");
            return;
        }

        if (currentIdx == goalIdx) {
            printPathAndDistance(goalIdx);
            analyzePathAndRecordTurns();     // Analyze and count turns immediately after the path is found
            return;
        }

        openList[currentIdx] = 0;
        closedList[currentIdx] = 1;

        int currentX, currentY;
        indexToXY(currentIdx, &currentX, &currentY);
        computeAndStoreDistances(currentIdx, currentX, currentY);  // Store distances as nodes are processed

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int newX = currentX + dx, newY = currentY + dy;
                if (!isValidPosition(newX, newY)) continue;

                int neighborIdx = xyToIndex(newX, newY);
                if (closedList[neighborIdx]) continue;

                double tentativeG = g[currentIdx] + calculateDistance(currentX, currentY, newX, newY);
                if (!openList[neighborIdx]) {
                    openList[neighborIdx] = 1;
                } else if (tentativeG >= g[neighborIdx]) {
                    continue;
                }

                parents[neighborIdx] = currentIdx;
                g[neighborIdx] = tentativeG;
                h[neighborIdx] = calculateDistance(newX, newY, goalX, goalY);
                f[neighborIdx] = g[neighborIdx] + h[neighborIdx];
            }
        }
    }
}


void buttonISR() // The ISR for the InterruptIn button. This function has minimal code for efficiency
{
    trigger_flag=true; // A single flag is set to indicate that the button has been pressed.
     // Use this time to perform new action
    printf("\rCurrent position: (%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
    printf("\rPerforming obstacles avoiding...");
    wait(5);
    printf("\rPerforming A star algorithm again...\n");
    wait(5);
    aStarSearch(start_x, start_y, globalCurrentNodeX, globalCurrentNodeY);
    printf("\rA star algorithm finished computing\n");
}

void resetPathfindingData() { // rest the array when obstacle detection arrived, clean it for new path storing 
    // Reset the parents array
    for (int i = 0; i < TOTAL_NODES; i++) {
        parents[i] = -1;
    }

    // Reset the closedList array
    for (int i = 0; i < TOTAL_NODES; i++) {
        closedList[i] = 0;
    }

    // Reset the g, h, and f arrays
    for (int i = 0; i < TOTAL_NODES; i++) {
        g[i] = LARGE_VALUE;
        h[i] = LARGE_VALUE;
        f[i] = LARGE_VALUE;
    }
}

void obstacleDetect(){ // Triggered when HC-SR04 measured distance under 15
    printf("\rObstalce detect\n");
    printf("\rCurrent position: (%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
    printf("\rPerforming obstacles avoiding...\n\r"); // turn right 90 degree or turn left 90 degree or take a U-turn sense if there is obstacles
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Direction dir = getDirection(previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
    switch(dir) {
        case NORTH: currentFacingDirection = "North"; break;
        case SOUTH: currentFacingDirection = "South"; break;
        case EAST: currentFacingDirection = "East"; break;
        case WEST: currentFacingDirection = "West"; break;
        case NORTH_EAST: currentFacingDirection = "North-East"; break;
        case NORTH_WEST: currentFacingDirection = "North-West"; break;
        case SOUTH_EAST: currentFacingDirection = "South-East"; break;
        case SOUTH_WEST: currentFacingDirection = "South-West"; break;
        default: currentFacingDirection = "Unknown"; break;
    }
    int testNodeX = globalCurrentNodeX;
    int testNodeY = globalCurrentNodeY;
    addObstacleAhead(globalCurrentNodeX, globalCurrentNodeY, currentFacingDirection); // add the obstacle detect infront to the obstacles array 
    if (strcmp(currentFacingDirection, "North") == 0){
        printf("\rFacing direction before avoiding: North\n\r");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX+1, testNodeY) == 1){
            globalCurrentNodeX+= 1;
            currentFacingDirection = "East";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX-2, testNodeY) == 1){
            globalCurrentNodeX-= 1;
            currentFacingDirection = "West";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "South") == 0){
        printf("\rFacing direction before avoiding: South\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX-1, testNodeY) == 1){
            globalCurrentNodeX-= 1;
            currentFacingDirection = "West";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX+2, testNodeY) == 1){
            globalCurrentNodeX+= 1;
            currentFacingDirection = "East";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "East") == 0){
        printf("\rFacing direction before avoiding: East\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX, testNodeY-1) == 1){
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX, testNodeY+2) == 1){
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX-= 1;
            currentFacingDirection = "West";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "West") == 0){
        printf("\rFacing direction before avoiding: West\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX, testNodeY+1) == 1){
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX, testNodeY-2) == 1){
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX+= 1;
            currentFacingDirection = "East";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "North-East") == 0){
        printf("\rFacing direction before avoiding: North-East\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX+1, testNodeY-1) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-East";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX-2, testNodeY+2) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-West";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX-= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-West";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "North-West") == 0){
        printf("\rFacing direction before avoiding: North-West\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX+1, testNodeY+1) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-East";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX-2, testNodeY-2) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-West";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX+= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-East";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "South-East") == 0){
        printf("\rFacing direction before avoiding: South-East\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX-1, testNodeY-1) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-West";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX+2, testNodeY+2) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-East";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX-= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-West";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "South-West") == 0){
        printf("\rFacing direction before avoiding: South-West\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX-1, testNodeY+1) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-West";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX+2, testNodeY-2) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-East";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX+= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-East";
            printf("\rMake a U-turn");
        }
    }
    else {
        printf("Unknown or no direction specified.\n");
    }

    wait(2);
    printf("\n\rPerforming A star algorithm again...\n");
    resetPathfindingData();
    wait(2);
    aStarSearch(start_x, start_y, globalCurrentNodeX, globalCurrentNodeY);

    goalIdx = xyToIndex(globalCurrentNodeX, globalCurrentNodeY);
    idx = goalIdx;

    printf("\rA star algorithm finished computing\n");
}

void runThePath(){
    button.rise(&buttonISR);
    int lastParentIdx = -1;

    while (parents[idx] != -1) {

            parentIdx = parents[idx];
            lastParentIdx = idx;
            
            globalCurrentNodeX = idx % MAX_COLS + 1;
            globalCurrentNodeY = idx / MAX_COLS + 1;
            float distance = sensorfront.distance(); // measures the distance from HC-SR04
            if (distance < 20){
                obstacleDetect();
                previousNodeX = -1;
                previousNodeY = -1;
                break;
            }

            printf("\r(%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
            // Skip direction calculation/printing for the first position

            if (previousNodeX != -1 && previousNodeY != -1) {
                Direction dir = getDirection(previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                switch(dir) {
                    case NORTH: currentFacingDirection = "North"; break;
                    case SOUTH: currentFacingDirection = "South"; break;
                    case EAST: currentFacingDirection = "East"; break;
                    case WEST: currentFacingDirection = "West"; break;
                    case NORTH_EAST: currentFacingDirection = "North-East"; break;
                    case NORTH_WEST: currentFacingDirection = "North-West"; break;
                    case SOUTH_EAST: currentFacingDirection = "South-East"; break;
                    case SOUTH_WEST: currentFacingDirection = "South-West"; break;
                    default: currentFacingDirection = "Unknown"; break;
                }
                printf("\rMoving from (%d, %d) to (%d, %d)\n", previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                printf("\rFacing direction: %s\n", currentFacingDirection);

                printf("\rDistance to nearest obstacle to the North: %d units\n", distanceToObstacleInDirection(globalCurrentNodeX, globalCurrentNodeY, NORTH));
                printf("\rDistance to nearest obstacle to the South: %d units\n", distanceToObstacleInDirection(globalCurrentNodeX, globalCurrentNodeY, SOUTH));
                printf("\rDistance to nearest obstacle to the East: %d units\n", distanceToObstacleInDirection(globalCurrentNodeX, globalCurrentNodeY, EAST));
                printf("\rDistance to nearest obstacle to the West: %d units\n", distanceToObstacleInDirection(globalCurrentNodeX, globalCurrentNodeY, WEST));

            }
            else {
                printf("Starting position: (%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
            }
            previousNodeX = globalCurrentNodeX;
            previousNodeY = globalCurrentNodeY;
            wait(1);
            idx = parentIdx;
            if (idx % MAX_COLS + 1 == start_x & idx / MAX_COLS + 1 == start_y){ // if reaches the final position then prints the final position
                globalCurrentNodeX = idx % MAX_COLS + 1;
                globalCurrentNodeY = idx / MAX_COLS + 1;
                Direction dir = getDirection(previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                char* currentFacingDirection;
                switch(dir) {
                    case NORTH: currentFacingDirection = "North"; break;
                    case SOUTH: currentFacingDirection = "South"; break;
                    case EAST: currentFacingDirection = "East"; break;
                    case WEST: currentFacingDirection = "West"; break;
                    case NORTH_EAST: currentFacingDirection = "North-East"; break;
                    case NORTH_WEST: currentFacingDirection = "North-West"; break;
                    case SOUTH_EAST: currentFacingDirection = "South-East"; break;
                    case SOUTH_WEST: currentFacingDirection = "South-West"; break;
                    default: currentFacingDirection = "Unknown"; break;
                }
                printf("\r(%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY); 
                printf("Moving from (%d, %d) to (%d, %d)\n", previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                printf("Facing direction: %s\n", currentFacingDirection);
                printf("\rDone\n");
            }
    }

}

void localization_demonstration(){
    float distancefront, distanceback, distanceright, distanceleft;
    distancefront = sensorfront.distance(); // measures the distance from HC-SR04
    distanceback = sensorback.distance(); // measures the distance from HC-SR04
    distanceright = sensorright.distance(); // measures the distance from HC-SR04
    distanceleft = sensorleft.distance(); // measures the distance from HC-SR04
    printf("\n\rCurrent sensorfront distance: %f cm\n", distancefront);
    printf("\rCurrent sensorback distance: %f cm\n", distanceback);
    printf("\rCurrent sensorright distance: %f cm\n", distanceright);
    printf("\rCurrent sensorleft distance: %f cm\n", distanceleft);
    locateCurrentPosition(distancefront, distanceback, distanceright, distanceleft, 3.0f);  // 5.0f is the tolerance
    wait(20);
}

int main() {
    printf("\rStarting\n");
    button.rise(&buttonISR);
    initObstacles();
    expandObstacles(); 
    printObstacleDimensions();
    aStarSearch(start_x, start_y, goal_x, goal_y); // Adjust the start and goal positions as needed
    int lastParentIdx = -1;
    int startIdx = xyToIndex(start_x, start_y); // Assuming these are global or properly initialized
    int goalIdx = xyToIndex(goal_x, goal_y);

    ///////////////////////////////test points for obstacle distance/////////////////////////////////////////////////////////////////////////////////////////////
    int x = 4; // Example position x
    int y = 11; // Example position y
    int distance;
    printf("\rNext turn at position (%d, %d):\n", x, y); // Prints the current position
    distance = distanceToObstacleInDirection(x, y, NORTH);
    printf("\rDistance to the nearest obstacle in North: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, EAST);
    printf("\rDistance to the nearest obstacle in East: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, WEST);
    printf("\rDistance to the nearest obstacle in West: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, SOUTH);
    printf("\rDistance to the nearest obstacle in South: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, NORTH_EAST);
    printf("\rDistance to the nearest obstacle in North-East: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, NORTH_WEST);
    printf("\rDistance to the nearest obstacle in North-West: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, SOUTH_EAST);
    printf("\rDistance to the nearest obstacle in South-East: %d units\n", distance);
    distance = distanceToObstacleInDirection(x, y, SOUTH_WEST);
    printf("\rDistance to the nearest obstacle in South-West: %d units\n", distance);
    wait(5);
    
    ///////////////////////////////test points for obstacle distance/////////////////////////////////////////////////////////////////////////////////////////////

    while(1) {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //runThePath(); // simulating environment without puzzlebot
            printTurnDetails(); // reach the turn point and perform action 
            //localization_demonstration(); //localization demonstration by placing obstacle in the distance of error +- 3 cm
            printf("\n\rEnd of trip!");
            myRawSerial.putc(static_cast<char>(stop));
            myRawSerial.putc('\n');  // Send newline character to mark end of message
            break;
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    }
    return 0;
}