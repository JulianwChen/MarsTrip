#pragma region VEXcode Generated Robot Configuration
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

brain Brain;

#define waitUntil(condition) \
    do                       \
    {                        \
        wait(5, msec);       \
    } while (!(condition))

#define repeat(iterations) \
    for (int iterator = 0; iterator < iterations; iterator++)

inertial BrainInertial = inertial();
motor LeftDriveSmart = motor(PORT1, 1, false);
motor RightDriveSmart = motor(PORT6, 1, true);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, BrainInertial, 200);

distance Distance4 = distance(PORT4);
optical Optical3 = optical(PORT3);

// Get random numbers from the inertial sensor
void initializeRandomSeed()
{
    wait(100, msec);
    double xAxis = BrainInertial.acceleration(xaxis) * 1000;
    double yAxis = BrainInertial.acceleration(yaxis) * 1000;
    double zAxis = BrainInertial.acceleration(zaxis) * 1000;
    int seed = int(xAxis + yAxis + zAxis);
    srand(seed);
}

// Callibrate the inertian sensor
bool vexcode_initial_drivetrain_calibration_completed = false;
void calibrateDrivetrain()
{
    wait(200, msec);
    Brain.Screen.print("Calibrating");
    Brain.Screen.newLine();
    Brain.Screen.print("Inertial");
    BrainInertial.calibrate();
    double t0 = Brain.timer(msec);
    while (BrainInertial.isCalibrating() && (Brain.timer(msec) - t0) < 5000.0)
    {
        wait(25, msec);
    }
    vexcode_initial_drivetrain_calibration_completed = true;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
}

// Run startup calibration
void vexcodeInit()
{
    calibrateDrivetrain();
    initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

// Constants used in the program
const double UNIT_MM = 20.0;
const int DRIVE_RPM = 10;
const int TURN_RPM = 10;
const int CALI_TURN_RPM = 15;

const double DETECT_MM = 40.0;

const int START_TILES = 0;

const double HEADING_N = 180.0;
const double HEADING_E = 270.0;

const double HEADING_TOL_DEG = 3.0;

const double WHEEL_TRAVEL_MM = 200.0;

const double IMU_CAL_TIMEOUT_MS = 8000.0;
const double CALI_SPIN_TIMEOUT_MS = 30000.0;
const double TURN_TIMEOUT_MS = 6000.0;
const double SETTLE_MS = 250.0;

const double MM_PER_MS = (DRIVE_RPM * WHEEL_TRAVEL_MM) / 60000.0;
const double DRIVE_TIMEOUT_MS = (UNIT_MM / MM_PER_MS) * 3.0 + 1500.0;

double overshootN = 0.0;
double overshootE = 0.0;

// Find the error between current heading and target heading
double heading_error(double target)
{
    double error = target - BrainInertial.heading(degrees);
    return fmod(error + 540.0, 360.0) - 180.0;
}

// Convert direction to heading
double heading_for(char direction)
{
    return (direction == 'N') ? HEADING_N : HEADING_E;
}

// Node
struct Node
{
    int id;
    double x, y;
    char angle;
    Node *next;
    Node *prev;
    Node(int id_, double x_, double y_, char ang_)
        : id(id_), x(x_), y(y_), angle(ang_), next(nullptr), prev(nullptr) {}
};

// Doubly Linked List
class WayPointList
{

private:
    // Remove a node from the list
    // Private since it is not called outside the class
    void removeNode(Node *node)
    {
        if (node->prev)
        {
            node->prev->next = node->next;
        }
        else
        {
            head = node->next;
        }
        if (node->next)
        {
            node->next->prev = node->prev;
        }
        else
        {
            tail = node->prev;
        }
        delete node;
        size--;
    }

public:
    Node *head;
    Node *tail;
    int size;
    int capacity;

    // Constructor
    WayPointList(int cap = 10)
        : head(nullptr), tail(nullptr), size(0), capacity(cap) {}

    // Destructor
    ~WayPointList()
    {
        Node *current = head;
        while (current)
        {
            Node *nextNode = current->next;
            delete current;
            current = nextNode;
        }
    }

    // Insert a new node at the end of the list
    void insert(int id, double x, double y, char angle)
    {
        Node *newNode = new Node(id, x, y, angle);
        if (!head)
        {
            head = tail = newNode;
        }
        else
        {
            tail->next = newNode;
            newNode->prev = tail;
            tail = newNode;
        }
        size++;
    }

    // Remove a node at a specific index
    void remove(int index)
    {
        if (index < 0 || index >= size)
        {
            return;
        }
        Node *current = head;
        for (int i = 0; i < index; i++)
        {
            current = current->next;
        }
        removeNode(current);
    }

    // Update the list
    // Remove consecutive nodes with id 0 and the same angle
    void update()
    {
        Node *current = head;
        while (current && current->next)
        {
            Node *nextNode = current->next;
            if (current->id == 0 && nextNode->id == 0 && current->angle == nextNode->angle)
            {
                removeNode(current);
            }
            current = nextNode;
        }
    }

    // Display the path on the brain screen
    void show(double x, double y, char heading) const
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("POS (%.0f,%.0f) %c", x, y, heading);

        Node *current = head;
        int row = 2;
        while (current && row <= 12)
        {
            Brain.Screen.setCursor(row, 1);
            Brain.Screen.print("id%d (%.0f,%.0f) %c",
                               current->id, current->x, current->y, current->angle);
            current = current->next;
            row++;
        }
    }
};

// Calibrate the inertial sensor
void cali_inertial()
{
    LeftDriveSmart.setVelocity(CALI_TURN_RPM, rpm);
    RightDriveSmart.setVelocity(CALI_TURN_RPM, rpm);
    BrainInertial.calibrate();

    double t0 = Brain.timer(msec);
    while (BrainInertial.isCalibrating() && (Brain.timer(msec) - t0) < IMU_CAL_TIMEOUT_MS)
    {
        wait(20, msec);
    }
    wait(1, seconds);

    Drivetrain.setTurnVelocity(CALI_TURN_RPM, rpm);
    Drivetrain.turn(left);
    wait(400, msec);

    t0 = Brain.timer(msec);
    while (fabs(BrainInertial.angle(degrees)) < 166.0 && (Brain.timer(msec) - t0) < CALI_SPIN_TIMEOUT_MS)
    {
        wait(10, msec);
    }

    Drivetrain.stop(brake);
    wait(1, seconds);
}

// Turn the robot to face a specific direction
void face(char direction)
{
    double heading = heading_for(direction);
    Drivetrain.setTurnVelocity(TURN_RPM, rpm);

    for (int attempt = 0; attempt < 2; attempt++)
    {
        Drivetrain.turnToHeading(heading, degrees, false);

        double t0 = Brain.timer(msec);
        while (Drivetrain.isTurning() && (Brain.timer(msec) - t0) < TURN_TIMEOUT_MS)
        {
            wait(10, msec);
        }
        Drivetrain.stop(brake);
        wait(SETTLE_MS, msec);

        if (fabs(heading_error(heading)) <= HEADING_TOL_DEG)
        {
            return;
        }
    }
}

// Read the distance from the distance sensor in millimeters
double read_distance_mm()
{
    return Distance4.objectDistance(mm);
}

// Scan forward three times and return the best distance reading
double scan_forward_mm()
{
    double best = -1.0;
    for (int i = 0; i < 3; i++)
    {
        double distance = read_distance_mm();
        if (distance > 0 && (best < 0 || distance < best))
            best = distance;
        wait(60, msec);
    }
    return best;
}

// Check if an object is detected within a certain distance
bool object_detected(double distance)
{
    return (distance > 0 && distance <= DETECT_MM);
}

// Check if there is an obstacle ahead
bool obstacle_ahead()
{
    wait(SETTLE_MS, msec);
    return object_detected(scan_forward_mm());
}

// Drive forward, stop if there is an obstacle
bool guarded_drive(char direction)
{
    double *bank = (direction == 'N') ? &overshootN : &overshootE;

    double target = UNIT_MM - *bank;
    if (target <= 1.0)
    {
        *bank = 0.0;
        return true;
    }

    double startDeg = LeftDriveSmart.position(degrees);

    Drivetrain.setDriveVelocity(DRIVE_RPM, rpm);
    Drivetrain.driveFor(forward, target, mm, false); // non-blocking

    bool blocked = false;

    double t0 = Brain.timer(msec);
    while (Drivetrain.isMoving() && (Brain.timer(msec) - t0) < DRIVE_TIMEOUT_MS)
    {
        if (object_detected(read_distance_mm()))
        {
            blocked = true;
            break;
        }
        wait(20, msec);
    }

    Drivetrain.stop(brake);
    wait(200, msec);

    if (blocked)
    {
        double travelledDeg = LeftDriveSmart.position(degrees) - startDeg;
        double travelledMM = (travelledDeg / 360.0) * WHEEL_TRAVEL_MM;
        if (travelledMM < 0.0)
        {
            travelledMM = 0.0;
        }
        *bank += travelledMM;
        if (*bank > UNIT_MM)
        {
            *bank = UNIT_MM;
        }
        return false;
    }

    *bank = 0.0;
    face(direction);
    return true;
}

// Pick the next direction to move in
char pick_direction(double x, double y, double xf, double yf)
{
    if (x >= xf)
    {
        return 'N';
    }
    if (y >= yf)
    {
        return 'E';
    }
    double dx = xf - x, dy = yf - y;
    return (dx <= dy) ? 'E' : 'N';
}

// Print current robot position and path
void print_state(WayPointList *path, double x, double y, char heading)
{
    path->show(x, y, heading);
}

// Update path display
void checkpoint(WayPointList *path, double x, double y, char heading)
{
    path->update();
    path->show(x, y, heading);
    wait(1, seconds);
}

// Record an obstacle in the path
void record_obstacle(WayPointList *path, double x, double y, char direction)
{
    double ox = x + (direction == 'E' ? 1 : 0);
    double oy = y + (direction == 'N' ? 1 : 0);
    path->insert(1, ox, oy, direction);
    checkpoint(path, x, y, direction);
}

// Attempt to move one step in the specified direction
bool try_step(WayPointList *path, double &x, double &y, char direction)
{
    face(direction);

    if (obstacle_ahead())
    {
        record_obstacle(path, x, y, direction);
        print_state(path, x, y, direction);
        return false;
    }

    if (!guarded_drive(direction))
    {
        record_obstacle(path, x, y, direction);
        print_state(path, x, y, direction);
        return false;
    }

    if (direction == 'E')
    {
        x += 1;
    }
    else
    {
        y += 1;
    }
    path->insert(0, x, y, direction);
    print_state(path, x, y, direction);
    return true;
}

// Drive along the path to the target until it reaches the goal
void run_robot(WayPointList *path, double xf, double yf, double startX, double startY, char startHeading)
{
    double x = startX, y = startY;
    char heading = startHeading;

    int attempts = 0;

    while (!(x == xf && y == yf) && attempts < 200)
    {
        attempts++;

        char direction = pick_direction(x, y, xf, yf);
        bool turning = (direction != heading);
        heading = direction;

        if (!try_step(path, x, y, direction))
        {
            char freeDirection = (direction == 'E') ? 'N' : 'E';
            heading = freeDirection;
            int guard = 0;
            while (guard < 40 && !(x == xf && y == yf))
            {
                guard++;
                if (!try_step(path, x, y, heading))
                {
                    break;
                }

                if ((heading == 'E' && x >= xf) || (heading == 'N' && y >= yf))
                {
                    break;
                }
            }
            checkpoint(path, x, y, heading);
            continue;
        }

        if (turning || path->size >= path->capacity)
        {
            checkpoint(path, x, y, heading);
        }
    }

    checkpoint(path, x, y, heading);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Mars pathed! (%.0f,%.0f)", x, y);
}

// main function
int main()
{
    vexcodeInit();

    cali_inertial();

    face('N');

    WayPointList path;
    double x = 0, y = (double)(-START_TILES);
    char heading = 'N';

    path.insert(0, x, y, heading);
    print_state(&path, x, y, heading);

    int guard = 0;
    while (y < 0 && guard < (START_TILES * 4 + 4))
    {
        if (!try_step(&path, x, y, 'N'))
        {
            try_step(&path, x, y, 'E');
        }
        guard++;
    }
    checkpoint(&path, x, y, heading);

    run_robot(&path, 18, 18, x, y, heading);
    wait(4, seconds);
    Brain.programStop();
    return 0;
}