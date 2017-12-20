#pragma once

#define FILE_ID "04"

//Fitness calculation
#define PENALTY_LINEAR_VEHICLES 0
#define PENALTY_TO_MANY_VEHICLES 0


#define BOARD_SIZE 1000;

//Poppuation
#define POPPULATION_SIZE 150

#define COMMUNICATION_RADIUS 200

//mutation rates
//rate=x is 1 in X chance. 0 = turnd off
#define MUTATION_INTRA_MOVE 6
#define MUTATION_SWAP 7
#define MUTATION_MOVE_SECTION 10
#define MUTATION_REVERSE 12
#define MUTATION_INTER_MOVE 3
#define MUTATION_INTER_MOVE_SECTION 3
#define MUTATION_INTER_SWAP 3
#define MUTATION_OPTIMAL_MOVE 0

#define MAX_INTER_MOVE_RELLATIVE_DIFF 10


//#define ELITEISM_USE 0
#define ELITE_MEMBERS ceil((double)POPPULATION_SIZE/100)

//Generation mix
//#define GENERATIONMIX_USE 1
#define USE_SIMPLE_DIFFERENCE 1
#define USE_CUSTOMMER_DIFFERENCE 1
//den med størst difference får trukket fra beste fitness/DIFFERENCE_SCALING
#define DIFFERENCE_SCALING 2  
#define RANDOM_SECTION POPPULATION_SIZE/2
#define NEW_RANDOM_INDIVIDUALS POPPULATION_SIZE*1/3 //går inn over random sectionen
#define CLONE_PENALTY 40

#define TOURNEY_SIZE 4
#define TOURNEY_ERROR_RATE 5 //probability of a worse solution winning

#define CLONE_RATE 15
#define IMPERFECT_CROSSOVER 10

#define GENERATION_CYCLES 100000

#define PRINT_RATE 500
#define SAVE_RATE 1000
//#define PRINT_RATE 1
//#define SAVE_RATE 1

#define SIMPLE_MODE 0