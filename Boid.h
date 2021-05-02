#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#pragma once

struct Boid {
	//Boid attributes
	olc::vf2d pos, vel, acc;
	int id;

	//Methods for moving the boids
	void update();
	olc::vf2d limitMagnitude(olc::vf2d a, float b);
	olc::vf2d setMagnitude(olc::vf2d a, float b);
};