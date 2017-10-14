#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
// struct semaphore *intersectionSem;
struct lock *lock;
struct cv *cv;
volatile int v[4][4] = {}; // initialize v with zeroes


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  cv = cv_create("cv");
  lock = lock_create("lock");
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  cv_destroy(cv);
  lock_destroy(lock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */
bool canGoNow(int, int); 

bool canGoNow(int o, int d){
  for(int i=0; i<4; ++i){
    for(int j=0; j<4; ++j){
      if(
	i != j && 
	v[i][j] != 0 && 
	!(i == o) &&
	!(i == d && j == o) &&
	!(d != j && ((j+1 % 4 == i) || (d+1 % 4 == o)))
        ){
        cv_wait(cv, lock);
        return false;
      } 
    }
  }

  v[o][d] += 1;
  return true;
}

void
intersection_before_entry(Direction origin, Direction destination) 
{
  lock_acquire(lock);

  while(!canGoNow(origin, destination)){} 

  lock_release(lock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  lock_acquire(lock);

  v[origin][destination] -= 1;
  cv_broadcast(cv, lock);

  lock_release(lock);
}



