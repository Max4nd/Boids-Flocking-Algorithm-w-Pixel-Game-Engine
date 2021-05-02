#include "Boid.h"
using namespace std;

//Move the boids
void Boid::update() {
	this->pos += limitMagnitude(this->vel, 0.5);
	this->vel += this->acc;
	this->acc = { 0,0 };
}

//Set the magnitude of a vector
olc::vf2d Boid::setMagnitude(olc::vf2d a, float b) {
	return a.norm() * b;
}

//Limit the magnitude of a vector
olc::vf2d Boid::limitMagnitude(olc::vf2d a, float b) {
	if (a.mag() > b)
		return a.norm() * b;
	else
		return a;
}

class Example : public olc::PixelGameEngine {
public:
	Example() {
		sAppName = "Boids - Flocking Simulation - Max Anderson";
	}

private:
	//Various attirbutes
	vector<Boid> Flock;
	Boid b;
	float alignMag = 4;
	float cohesionMag = 4;
	float separationMag = 4;
	int vicinity = 50;

	//Add a boid to the vector
	void addBoid(float x, float y) {
		float rone = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float rtwo = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		b.pos = { x,y };
		b.vel = { rone, rtwo };
		b.acc = { 0.0 ,0.0 };
		b.id = Flock.size();
		Flock.emplace_back(b);
	}

	//Calculate the distance between 2 boids
	float distanceBetween(olc::vf2d a, olc::vf2d b) {
		return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) * 1.0);
	}

public:
	bool OnUserCreate() override {
		//Add 100 boids in random locations with random velocities
		for (int i = 0; i < 100; i++)
			addBoid(rand() % ScreenWidth(), rand() % ScreenHeight());

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override {

		//Clear the screen
		Clear(olc::BLACK);

		//Draw mag values to screen
		DrawString(0, 0, "Align Force: " + std::to_string(alignMag), olc::WHITE, 3);
		DrawString(0, 25, "Cohesion Force: " + std::to_string(cohesionMag), olc::WHITE, 3);
		DrawString(0, 50, "Separation Force: " + std::to_string(separationMag), olc::WHITE, 3);

		//Change the magnitudes of each algorithm
		// Align (Q to add, W to minus)
		// Cohesion (A to add, S to minus)
		// Separation (Z to add, X to minus)
		if (GetKey(olc::Key::Q).bHeld)
			alignMag -= 0.01;
		if (GetKey(olc::Key::W).bHeld)
			alignMag += 0.01;
		if (GetKey(olc::Key::A).bHeld)
			cohesionMag -= 0.01;
		if (GetKey(olc::Key::S).bHeld)
			cohesionMag += 0.01;
		if (GetKey(olc::Key::Z).bHeld)
			separationMag -= 0.01;
		if (GetKey(olc::Key::X).bHeld)
			separationMag += 0.01;

		//Find the steering vectors for each algorithm, then add them to the boids acceleration
		for (auto& b1 : Flock) {
			olc::vf2d steeringA;
			olc::vf2d steeringC;
			olc::vf2d steeringS;
			int total = 0;
			for (auto& b2 : Flock) {
				float distance = distanceBetween(b1.pos, b2.pos);
				olc::vf2d diff = b1.pos - b2.pos;
				if (b1.id != b2.id && distance < vicinity) {
					steeringA += b2.vel;
					steeringC += b2.pos;
					steeringS += diff;
					total++;
				}
			}
			if (total > 0) {
				//Align
				steeringA /= total;
				steeringA = b1.setMagnitude(steeringA, alignMag);
				steeringA -= b1.vel;
				steeringA = b1.limitMagnitude(steeringA, 1);
				b1.acc += steeringA;
				//Cohesion
				steeringC /= total;
				steeringC -= b1.pos;
				steeringC = b1.setMagnitude(steeringC, cohesionMag);
				steeringC -= b1.vel;
				steeringC = b1.limitMagnitude(steeringC, 1);
				b1.acc += steeringC;
				//Separation
				steeringS /= total;
				steeringS = b1.setMagnitude(steeringS, separationMag);
				steeringS -= b1.vel;
				steeringS = b1.limitMagnitude(steeringS, 1);
				b1.acc += steeringS;
			}

			//Edges
			if (b1.pos.x > ScreenWidth())
				b1.pos.x = 0;
			if (b1.pos.x < 0)
				b1.pos.x = ScreenWidth();
			if (b1.pos.y > ScreenHeight())
				b1.pos.y = 0;
			if (b1.pos.y < 0)
				b1.pos.y = ScreenHeight();

			//Draw the boids and update the postions
			DrawCircle(b1.pos, 2, olc::GREEN);
			b1.update();
		}

		return true;
	}
};

int main() {
	Example demo;
	//Run
	if (demo.Construct(768, 768, 1, 1, false))
		demo.Start();

	return 0;
}