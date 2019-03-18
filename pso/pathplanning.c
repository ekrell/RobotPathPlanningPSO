#include <math.h>
#include <string.h>
#include <gsl/gsl_rng.h>
#include "pso.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <ctype.h>

typedef struct {
    int ID; 
    double position_coords[2];
    double target_coords[2];
    double distToTarget;
    double stepSize;
    double velocity;
} robot_t;

typedef struct {
    double *waypoints;
    int numWaypoints;
} path_t;

typedef struct {
    double mins[2];
    double maxs[2];
    int ** map;
} env_t;

typedef struct {
    double start[2];
    double stop[2];
    double c1;
    double c2;
    int w_strategy;
    double w_min;
    double w_max;
    int nhood_topology;
    double nhood_size;
    env_t * env;
} pso_params_t;

int ** readMap (char * fhandle, int height, int width) {

    FILE *file;
    file = fopen(fhandle, "r");
    size_t count; 
    char *line = (char *) malloc (sizeof (char) * width + 1);
    
    int i = 0, j = 0, ylim = 0;
    int ** map = (int **) malloc (sizeof (int *) * height);
    while (getline (&line, &count, file) != -1 && ylim < height){
        map[i] = (int *) malloc (sizeof (int) * width);
        for (j = 0; j < width; j++) {
            map[i][j] = line[j] - '0';
        }
        i++;
        ylim++;
    }
    fclose(file);
    return map;
}

void printMap (int **map, int height, int width){
    int i = 0, j = 0;
    for (i = 0; i < height; i++){
        for (j = 0; j < width; j++){
            printf ("%d", map[i][j]);
        }
        printf ("\n");
    }

}

double euclideanDistance(double xi, double yi, double xj, double yj) {
    return pow( pow(xi - xj, 2) + pow(yi - yj, 2), 0.5);
}

double ChessboardDistance(double xi, double yi, double xj, double yj) {
    double xDiff = abs(xi - xj);
    double yDiff = abs(yi - yj);
    double max = 0.0;
    if (xDiff > yDiff){
        max = xDiff;
    }
    else {
        max = yDiff;
    }
    return max;
}

robot_t * initRobot(int ID, double xInit, double yInit, double xTarget, double yTarget, double stepSize, double velocity) {
    robot_t *robot = (robot_t *) malloc (sizeof (robot_t) * 1);
    robot->ID = ID;
    robot->position_coords[0] = xInit;
    robot->position_coords[1] = yInit;
    robot->target_coords[0] = xTarget;
    robot->target_coords[1] = yTarget;
    robot->distToTarget = euclideanDistance(xInit, yInit, xTarget, yTarget);
    robot->stepSize = stepSize;
    // This is the velocity of the robot
    robot->velocity = velocity;
    return robot;
}

env_t * initEnv(double xMin, double yMin, double xMax, double yMax, int ** map) {
    env_t *env = (env_t *) malloc (sizeof (env_t) * 1);
    env->mins[0] = xMin;
    env->mins[1] = yMin;
    env->maxs[0] = xMax;
    env->maxs[1] = yMax;
    env->map = map;
    return env;
}

void printRobot(robot_t *robot) {
    printf ("Robot %d is at (%f, %f).\n", robot->ID, robot->position_coords[0], robot->position_coords[1]);
    printf ("The goal is at (%f, %f).\n", robot->target_coords[0], robot->target_coords[1]);
    printf ("It moves with velocity %f and with step size %f.\n", robot->velocity, robot->stepSize);
    printf ("It is currently at distance %f from the target.\n", robot->distToTarget);
}

void printEnv(env_t *env) {
    printf ("The world is of size %f x %f.\n", (env->maxs[0] - env->mins[0]), (env->maxs[1] - env->mins[1]));
    printf ("The bottom-left point is (%f, %f).\n", env->mins[0], env->mins[1]);
    printf ("The top-right point is (%f, %f).\n", env->maxs[0], env->maxs[1]);
}

int line2 (int x0, int y0, int x1, int y1, int ** map, int xLimit, int yLimit) { 
// Source: https://github.com/ssloy/tinyrenderer/wiki/Lesson-1:-Bresenham%E2%80%99s-Line-Drawing-Algorithm

    int count = 0;
    float t = 0.0;
	for (t=0.; t<1.; t+=.01) { 
        int x = x0*(1.-t) + x1*t; 
        int y = y0*(1.-t) + y1*t; 

		if (x < yLimit && y < xLimit) {
			if (map[x][y] > 0){
				count++;
			}
		}
    } 
	return count;
}

int pso_path_countObstructions(double *vec, int dim, void *params) {
	pso_params_t * parameters;
    parameters = (pso_params_t *) params;
    
    // Ensure even-length vector
    if (dim % 2 != 0) {
        printf ("Inside 'pso_path_countObstruction': solution vector must be even-length!\n");
        exit (1);
    }

    int penaltyCount = 0;
    int i, count = 0;
    double xi = vec[0];
    double yi = vec[1];
    double xj = 0, yj = 0;

	penaltyCount = penaltyCount + line2 (parameters->start[0], parameters->start[1], 
            xi, yi, parameters->env->map, parameters->env->maxs[0], parameters->env->maxs[1]);

    count = 2;
    int iterations = dim / 2 - 1;
    for (i=0; i<iterations;i++) {
        xj = vec[count];
        count++;
        yj = vec[count];
        count++;
        penaltyCount = penaltyCount + line2 (xi, yi, xj, yj, parameters->env->map, parameters->env->maxs[0], parameters->env->maxs[1]);
        xi = xj;
        yi = yj;
    }

    penaltyCount = penaltyCount + line2 (xj, yj, parameters->stop[0], parameters->stop[1], 
            parameters->env->map, parameters->env->maxs[0], parameters->env->maxs[1]);
    return penaltyCount * 10;
}

double pso_path_penalty(double *vec, int dim, void *params) {
    // Penalizes the presence of obstacles in the path
    int countObstructions = pso_path_countObstructions (vec, dim, params);
    return pow (countObstructions, 1);
}

double pso_path(double *vec, int dim, void *params) {
    pso_params_t * parameters;
    parameters = (pso_params_t *) params;

    // Ensure even-length vector
    if (dim % 2 != 0) {
        printf ("Inside 'pso_path': solution vector must be even-length!\n");
        exit (1);
    }

    double distance = 0.0;
    int i, count = 0;

    double xi = vec[0];
    double yi = vec[1];
    double xj, yj = 0;

    distance = distance + euclideanDistance(parameters->start[0], parameters->start[1], xi, yi);

    count = 2;
    int iterations = dim / 2 - 1;
    for (i=0; i<iterations;i++) {
        xj = vec[count];
        count++;
        yj = vec[count];
        count++;
        distance = distance + euclideanDistance(xi, yi, xj, yj);
        xi = xj;
        yi = yj;
    }

    distance = distance + euclideanDistance(xj, yj, parameters->stop[0], parameters->stop[1]);
	distance = distance + pso_path_penalty(vec, dim, params);

    return distance;
}

void pso_set_path_settings(pso_settings_t *settings, 
        pso_params_t *params, env_t *env, robot_t *robot, int waypoints) {
    /* WARNING */
    // Only valid if a square environment with same start and stop 
    // EX: (0, 0) to (100, 100) because the pso lib
    // only considers each as an 'x-value', not knowing that
    // we are using a vector where odds are 'x' and evens are 'y'
    settings->x_lo = env->mins[0];
    settings->x_hi = env->maxs[0];

    settings->limits = pso_autofill_limits(settings->x_lo, settings->x_hi, settings->dim);

    settings->dim = waypoints * 2;
    // Set odd values limits using X-min and X-max
    // and even values limits using Y-min and Y-max
    int i;
    int count = 0;
    for (i = 0; i < waypoints; i++) {
        settings->limits[0][count] = env->mins[0];
        settings->limits[1][count] = env->maxs[0];
        count++;
        settings->limits[0][count] = env->mins[1];
        settings->limits[1][count] = env->maxs[1];
        count++;
    }

    pso_print_limits(settings->limits, settings->dim);

    // Set parameters, if not default
    if (params->c1 >= 0)
        settings->c1 = params->c1;
    if (params->c2 >= 0)
        settings->c2 = params->c2;
    if (params->w_min >= 0)
        settings->w_min = params->w_min;
    if (params->w_max >= 0)
        settings->w_max = params->w_max;
    if (params->w_strategy >= 0)
        settings->w_strategy = params->w_strategy;
    if (params->nhood_size >= 0)
        settings->nhood_size = params->nhood_size;
    if (params->nhood_topology >= 0)
        settings->nhood_strategy = params->nhood_topology;

    settings->goal = 1e-5;
    settings->numset = INTEGER;
}

int getPSOParam_w_stategy(int code){
    if (code == 0)
        return PSO_W_CONST;
    if (code == 1)
        return PSO_W_LIN_DEC;
    return code;
}

int getPSOParam_nhood_topology(int code){
    if (code == 0)
        return PSO_NHOOD_GLOBAL;
    if (code == 1)
        return PSO_NHOOD_RING;
    if (code == 2)
        return PSO_NHOOD_RANDOM;
    return code;
}

int main (int argc, char **argv){

    /* Start nasty hard coded segment */
    int inRoboID = 0;
    double inStartX = 70.0;
    double inStartY = 70.0;
    double inEndX = 136.0;
    double inEndY = 127.0;
    double inStepSize = 1;
    double inVelocity = 2;
    double inOriginX = 0;
    double inOriginY = 0;
    double inHorizonX = 200;
    double inHorizonY = 200;  // 70
    //char inFileHandle[20] = "maps/sampleMap4.dat\0";
    char inFileHandle[] = "OccupancyMap.txt";
    int waypoints = 5;
    /* End nasty hard coded segment */

    /* PSO parameters */
    double pso_c1 = -1.0;
    double pso_c2 = -1.0;
    double pso_w_max = -1.0;
    double pso_w_min = -1.0;
    int pso_w_strategy_select = -1;
    int pso_nhood_size = -1;
    int pso_nhood_topology_select = -1;

    int pso_w_strategy = -1;
    int pso_nhood_topology = -1;

    /* Option parsing */
    int verbose = 0;
    char *inFileHandlePtr = NULL;

    int c;
    opterr = 0;

    while ((c = getopt (argc, argv, "a:b:c:d:e:f:n:m:p:q:r:s:t:w:x:v")) != -1)
        switch (c) {
            case 'v':
                verbose = 1;
                break;
            case 'a':
                sscanf(optarg, "%lf", &inHorizonX);
                break;
            case 'b':
                sscanf(optarg, "%lf", &inHorizonY);
                break;
            case 'c':
                sscanf(optarg, "%lf", &inStartX);
                break;
            case 'd':
                sscanf(optarg, "%lf", &inStartY);
                break;
            case 'e': 
                sscanf(optarg, "%lf", &inEndX);
                break;
            case 'f':
                sscanf(optarg, "%lf", &inEndY);
                break;
            case 'n':
                sscanf(optarg, "%d", &waypoints);
                break;
            case 'm':
                inFileHandlePtr = optarg;
                break;
            case 'p': /* PSO c1 */
                sscanf(optarg, "%lf", &pso_c1);
                break;
            case 'q': /* PSO c2 */
                sscanf(optarg, "%lf", &pso_c2);
                break;
            case 'r': /* PSO w_max */
                sscanf(optarg, "%lf", &pso_w_max);
                break;
            case 's': /* PSO w_min */
                sscanf(optarg, "%lf", &pso_w_min);
                break;
            case 't': /* PSO w_strategy */
                sscanf(optarg, "%d", &pso_w_strategy_select);
                break;
            case 'w': /* PSO nhood_strategy */
                sscanf(optarg, "%d", &pso_nhood_topology_select);
                break;
            case 'x': /* PSO nhood_size */
                sscanf(optarg, "%d", &pso_nhood_size);
                break;
            default:
                abort();
        }

    // PSO options from user selection
    pso_w_strategy     = getPSOParam_w_stategy(pso_w_strategy_select);
    pso_nhood_topology = getPSOParam_nhood_topology(pso_nhood_topology_select);

    // Print argument options
    printf ("Dimension = (%f,%f), Start = (%f,%f), Target = (%f,%f)\n", 
            inHorizonX, inHorizonY, inStartX, inStartY, inEndX, inEndY);
    printf ("Map File = %s\n", inFileHandlePtr);
    printf ("PSO: c1 = %f, c2 = %f, weight strategy = %d, neighborhood topology = %d\n", 
            pso_c1, pso_c2, pso_w_strategy, pso_nhood_topology);
    if (pso_w_strategy == PSO_W_LIN_DEC)
        printf ("\tweight min = %f, weight max = %f\n", pso_w_min, pso_w_max);
    if (pso_nhood_topology_select == PSO_NHOOD_RANDOM)
        printf("\tneighborhood size = %d\n", pso_nhood_size);

    /* Read occupancy map */
    int ** map = readMap (inFileHandlePtr, inHorizonY, inHorizonX);

    /* Initialize robot */
    robot_t * robot = initRobot(inRoboID, inStartX, inStartY, inEndX, inEndY, inStepSize, inVelocity);
    printRobot(robot);

    /* Init pso objecttive function params */
    pso_params_t * pso_params;
    pso_params = malloc (sizeof (pso_params_t) * 1);
    pso_params->env = initEnv(inOriginX, inOriginY, inHorizonX, inHorizonY, map);
    pso_params->start[0] = robot->position_coords[0];
    pso_params->start[1] = robot->position_coords[1];
    pso_params->stop[0] = robot->target_coords[0];
    pso_params->stop[1] = robot->target_coords[1];
    pso_params->c1 = pso_c1;
    pso_params->c2 = pso_c2;
    pso_params->w_strategy = pso_w_strategy;
    pso_params->w_min = pso_w_min;
    pso_params->w_max = pso_w_max;
    pso_params->nhood_topology = pso_nhood_topology;
    pso_params->nhood_size = pso_nhood_size;
    printEnv(pso_params->env);

    /* Init pso settings */
    pso_settings_t settings;
    // Set the default settings
    pso_set_default_settings(&settings);
    
    /* PSO settings */
    //int maxIterations = 500;
    int popSize = 100;

    /* Use pso library */
    // Define objective function
    pso_obj_fun_t obj_fun = pso_path;
    // Set the problem specific settings
    pso_set_path_settings(&settings, pso_params, pso_params->env, robot, waypoints);
//settings.size = popSize;
//settings.nhood_strategy = PSO_NHOOD_RING;
    settings.dim = waypoints * 2;
//settings.nhood_size = 10;
//settings.w_strategy = PSO_W_LIN_DEC;
    settings.steps = 100000;
    settings.print_every = 10;
    // Init global best solution
    pso_result_t solution;
    // Allocate mem for best position buffer
    solution.gbest = malloc (settings.dim * sizeof(double));
    // Run pso algorithm
    pso_solve(obj_fun, pso_params, &solution, &settings);
    
    // Display best result
    int i, count = 0;
    printf ("Solution waypoints:\n");
    for(i=0;i<settings.dim/2;i++){
        printf ("(%f, %f)\n", solution.gbest[count], solution.gbest[count + 1]);
        count = count + 2;
    }
    printf("Solution distance: %f\n", solution.error);


    int obstacles = pso_path_countObstructions(solution.gbest, settings.dim, pso_params);
    printf ("obstacles: %d\n", obstacles);

	

    // Free global best buffer
    free(solution.gbest);

    return 0;
}

