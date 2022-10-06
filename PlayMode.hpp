#include "Mode.hpp"

#include "Scene.hpp"
#include "WalkMesh.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>

struct PlayMode : Mode {
	PlayMode();
	virtual ~PlayMode();

	//functions called by main loop:
	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;

	//----- game state -----

	//input tracking:
	struct Button {
		uint8_t downs = 0;
		uint8_t pressed = 0;
	} left, right, down, up, interact;

	//local copy of the game scene (so code can change it during gameplay):
	Scene scene;

	//player info:
	struct Player {
		WalkPoint at;
		//transform is at player's feet and will be yawed by mouse left/right motion:
		Scene::Transform *transform = nullptr;
		//camera is at player's head and will be pitched by mouse up/down motion:
		Scene::Camera *camera = nullptr;
	} player;


	// Game vars
	struct House {
		glm::vec3 dropoffPos;
		Scene::Drawable *drawable = nullptr;
	};

	std::vector<House*> houses;

	uint32_t score = 0;
	float max_timer;
	float timer; // timer in seconds

	int currentHouseIndex = 0;
	float distThreshold = 4.0f;

	Scene::Drawable *marker = nullptr;
	glm::vec3 marker_base_position;

	bool gameOver = false;

	// Generates random integer [min, max)
	size_t rand_int(size_t min, size_t max);

	// Generates random float [min, max]
	float rand_float(float lo, float hi);

	// Picks the next house for the dropoff
	void pick_house();
};
