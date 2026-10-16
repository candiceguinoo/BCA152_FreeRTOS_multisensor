#ifndef MOTION_H
#define MOTION_H

// Configure the PIR input. Call once from app_main.
void motionInit();

// MotionTask: polls the PIR every 100 ms, maintains the inactivity
// timer, and drives the ACTIVE/INACTIVE state and event bits.
void motionTask(void *parameter);

#endif // MOTION_H