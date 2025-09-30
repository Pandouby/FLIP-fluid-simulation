/**         Includes         */
#include <ConfigPlatform.h>
#include <we_print.h>
#include "ICLED.h"
#include "ICLED_demos.h"
#include <math.h>
#include <vector>
#include <cmath>
#include <cstdint>

/* Test Modes */
typedef enum
{
    TEST1,
    TEST2,
    TEST3,
    TEST4,
    TEST5,
    TEST6,
    TEST7,
    TEST8,
    TEST_Total_Count
} TestMode;

static volatile TestMode current_mode = TEST2;

static void PROG_ISR_handler();

static unsigned long prog_debounce_time_elapsed = 0;

static bool running_loop = true;

static volatile bool initial_test_run = true;

static bool orientation_changed = false;

// fluid const
const float density = 1000.0;
const int fNumX = 16;
const int fNumY = 8;
const int h = 1;
const int fInvSpacing = h;
const int fNumCells = fNumX * fNumY;

const int U_FIELD = 0;
const int V_FIELD = 1;

const int FLUID_CELL = 0;
const int AIR_CELL = 1;
const int SOLID_CELL = 2;

const int cnt = 0;

float clamp(float x, float min, float max)
{
    if (x < min)
        return min;
    else if (x > max)
        return max;
    else
        return x;
}

/*
    gravity : -9.81, // Is going to get replaced with acelerometer input
    dt : 1.0 / 120.0,
    flipRatio : 0.9,
    numPressureIters : 100,
    numParticleIters : 2,
    frameNr : 0,
    overRelaxation : 1.9,
    compensateDrift : true,
    separateParticles : true,
    obstacleX : 0.0,
    obstacleY : 0.0,
    obstacleRadius: 0.15,
    paused: true,
    showObstacle: true,
    obstacleVelX: 0.0,
    obstacleVelY: 0.0,
    showParticles: true,
    showGrid: false,
    fluid: null
*/

void setup()
{
    if (!ICLED_Init(RGB, Landscape))
    {
        Serial.println("ICLED Init failed");
    }

    pinMode(ICLED_PROG_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ICLED_PROG_PIN), PROG_ISR_handler, FALLING);
}

void loop()
{
    ICLED_set_color_system(RGB);

    int grid[7][15];

    ICLED_clear(false);
    initial_test_run = false;
    ICLED_set_pixel(5, 50, 0, 50, 128);
    ICLED_set_screen_pixel(7, 0, 50, 50, 50, 128);

    ICLED_set_char('2', 0, 128, 128, 128, 50);
}

static void PROG_ISR_handler()
{
    if ((millis() - prog_debounce_time_elapsed) < 300)
    {
        return;
    }
    prog_debounce_time_elapsed = millis();
    current_mode = (TestMode)((current_mode + 1) % TEST_Total_Count);
    running_loop = false;
    initial_test_run = true;
}

class FlipFluid
{
public:
    float density;

    int fNumX, fNumY;
    float h;
    float fInvSpacing;
    int fNumCells;

    // fluid arrays
    std::vector<float> u, v, du, dv, prevU, prevV, p, s;
    std::vector<int32_t> cellType;

    // particles
    int maxParticles;
    std::vector<float> particlePos;
    std::vector<float> particleColor;
    std::vector<float> particleVel;
    std::vector<float> particleDensity;
    float particleRestDensity;

    float particleRadius;
    float pInvSpacing;
    int pNumX, pNumY, pNumCells;

    std::vector<int32_t> numCellParticles; // Particles per cell
    std::vector<int32_t> firstCellParticle;
    std::vector<int32_t> cellParticleIds;

    int numParticles;

    FlipFluid(float density_,
              float width,
              float height,
              float spacing,
              float particleRadius_,
              int maxParticles_)
        : density(density_),
          particleRestDensity(0.0f),
          particleRadius(particleRadius_),
          maxParticles(maxParticles_),
          numParticles(0)
    {
        // Grid setup
        fNumX = static_cast<int>(std::floor(width / spacing)) + 1;
        fNumY = static_cast<int>(std::floor(height / spacing)) + 1;
        h = std::max(width / fNumX, height / fNumY);
        fInvSpacing = 1.0f / h;
        fNumCells = fNumX * fNumY;

        // Allocate fluid arrays
        u.resize(fNumCells);
        v.resize(fNumCells);
        du.resize(fNumCells);
        dv.resize(fNumCells);
        prevU.resize(fNumCells);
        prevV.resize(fNumCells);
        p.resize(fNumCells);
        s.resize(fNumCells);
        cellType.resize(fNumCells);

        // Particles
        particlePos.resize(2 * maxParticles);
        particleColor.resize(3 * maxParticles, 0.0f);
        for (int i = 0; i < maxParticles; ++i)
            particleColor[3 * i + 2] = 1.0f;

        particleVel.resize(2 * maxParticles);
        particleDensity.resize(fNumCells);

        // Particle grid
        pInvSpacing = 1.0f / (2.2f * particleRadius_);
        pNumX = static_cast<int>(std::floor(width * pInvSpacing)) + 1;
        pNumY = static_cast<int>(std::floor(height * pInvSpacing)) + 1;
        pNumCells = pNumX * pNumY;

        numCellParticles.resize(pNumCells);
        firstCellParticle.resize(pNumCells + 1);
        cellParticleIds.resize(maxParticles);
    }

    void intergateParticles(float dt, float gravity)
    {
        for (int i = 0; i < numParticles; i++)
        {
            particleVel[2 * i + 1] += dt * gravity;
            particlePos[2 * i] += particleVel[2 * i] * dt;
            particlePos[2 * i + 1] += particleVel[2 * i + 1] * dt;
        }
    }

    void pushParticlesApart(int numIters)
    {
        // count particles per cell

        std::fill(numCellParticles.begin(), numCellParticles.end(), 0);

        for (int i = 0; i < numParticles; i++)
        {
            float x = particlePos[2 * i];
            float y = particlePos[2 * i + 1];

            float xi = clamp(std::floor(x * pInvSpacing), 0, pNumX - 1);
            float yi = clamp(std::floor(y * pInvSpacing), 0, pNumY - 1);
            float cellNr = xi * pNumY + yi; // -> look into this and understand
            numCellParticles[cellNr]++;
        }

        // partial sums

        int first = 0;

        for (int i = 0; i < pNumCells; i++)
        {
            first += numCellParticles[i];
            firstCellParticle[i] = first;
        }
        firstCellParticle[pNumCells] = first; // guard

        // fill particles into cells

        for (int i = 0; i < numParticles; i++)
        {
            float x = particlePos[2 * i];
            float y = particlePos[2 * i + 1];

            int xi = clamp(std::floor(x * pInvSpacing), 0, pNumX - 1);
            int yi = clamp(std::floor(y * pInvSpacing), 0, pNumY - 1);
            int cellNr = xi * pNumY + yi;
            firstCellParticle[cellNr]--;
            cellParticleIds[firstCellParticle[cellNr]] = i;
        }

        // push particles apart

        float minDist = 2.0 * particleRadius;
        float minDist2 = minDist * minDist;

        for (int iter = 0; iter < numIters; iter++)
        {

            for (int i = 0; i < numParticles; i++)
            {
                float px = particlePos[2 * i];
                float py = particlePos[2 * i + 1];

                int pxi = std::floor(px * pInvSpacing);
                int pyi = std::floor(py * pInvSpacing);
                int x0 = std::max(pxi - 1, 0);
                int y0 = std::max(pyi - 1, 0);
                int x1 = std::min(pxi + 1, pNumX - 1);
                int y1 = std::min(pyi + 1, pNumY - 1);

                for (int xi = x0; xi <= x1; xi++)
                {
                    for (int yi = y0; yi <= y1; yi++)
                    {
                        int cellNr = xi * pNumY + yi;
                        int first = firstCellParticle[cellNr];
                        int last = firstCellParticle[cellNr + 1];
                        for (int j = first; j < last; j++)
                        {
                            int id = cellParticleIds[j];
                            if (id == i)
                                continue;
                            float qx = particlePos[2 * id];
                            float qy = particlePos[2 * id + 1];

                            float dx = qx - px;
                            float dy = qy - py;
                            float d2 = dx * dx + dy * dy;
                            if (d2 > minDist2 || d2 == 0.0)
                                continue;
                            float d = std::sqrt(d2);
                            float s = 0.5 * (minDist - d) / d;
                            dx *= s;
                            dy *= s;
                            particlePos[2 * i] -= dx;
                            particlePos[2 * i + 1] -= dy;
                            particlePos[2 * id] += dx;
                            particlePos[2 * id + 1] += dy;
                        }
                    }
                }
            }
        }
    }

    void handleParticleCollisions(float obstacleX, float obstacleY, float obstacleRadius) 
		{
		    int h = 1.0 / fInvSpacing;
			float r = particleRadius;
			float obr = obstacleRadius;
			float obr2 = or * or;
			float minDist = obstacleRadius + r;
			float minDist2 = minDist * minDist;

			float minX = h + r;
			float maxX = (fNumX - 1) * h - r;
			float minY = h + r;
			float maxY = (fNumY - 1) * h - r;


			for (var i = 0; i < this.numParticles; i++) {
				var x = this.particlePos[2 * i];
				var y = this.particlePos[2 * i + 1];

				var dx = x - obstacleX;
				var dy = y - obstacleY;
				var d2 = dx * dx + dy * dy;

				// obstacle collision

				if (d2 < minDist2) {

					// var d = Math.sqrt(d2);
					// var s = (minDist - d) / d;
					// x += dx * s;
					// y += dy * s;

					this.particleVel[2 * i] = scene.obstacleVelX;
					this.particleVel[2 * i + 1] = scene.obstacleVelY;
				}

				// wall collisions

				if (x < minX) {
					x = minX;
					this.particleVel[2 * i] = 0.0;

				}
				if (x > maxX) {
					x = maxX;
					this.particleVel[2 * i] = 0.0;
				}
				if (y < minY) {
					y = minY;
					this.particleVel[2 * i + 1] = 0.0;
				}
				if (y > maxY) {
					y = maxY;
					this.particleVel[2 * i + 1] = 0.0;
				}
				this.particlePos[2 * i] = x;
				this.particlePos[2 * i + 1] = y;
			}
		}

};