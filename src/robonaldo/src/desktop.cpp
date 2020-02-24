#include <SDL2/SDL.h>
#include "ros/ros.h"
#include "robonaldo/keyboard_input.h"
#include <sstream>
#include <termios.h>
#include <stdio.h>
#include <SDL2/SDL.h>

const int windowWidth = 640;
const int windowHeight = 480;

SDL_Window* window;
SDL_Renderer* renderer;
bool windowOpen = true;

void initSDL() 
{

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
    fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
  }

  window = SDL_CreateWindow("Robot Control", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, windowWidth, windowHeight, 0);

  if (window == nullptr) {
    fprintf(stderr, "could not create window: %s\n", SDL_GetError());
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
}

void shutDownSDL(){
  SDL_DestroyRenderer(renderer);
  renderer = nullptr;

  SDL_DestroyWindow(window);
  window = nullptr;

  SDL_Quit();
}

/* Get user input and set msg accordingly, using WASD */
void getInput(robonaldo::keyboard_input& msg) {
  
  SDL_Event event;
  while (SDL_PollEvent(&event)) {

    if (event.type == SDL_QUIT) {
      windowOpen = false;
    }

  }

  const Uint8* state = SDL_GetKeyboardState(nullptr);

  // Check if excape was hit.
  if (state[SDL_SCANCODE_ESCAPE]) {
    windowOpen = false;
  }

  msg.up = state[SDL_SCANCODE_W];
  msg.left = state[SDL_SCANCODE_A];
  msg.down = state[SDL_SCANCODE_S];
  msg.right = state[SDL_SCANCODE_D];

}


int main(int argc, char **argv) {

  initSDL();

	ros::init(argc, argv, "desktop");
	ros::NodeHandle n;
	ros::Publisher user_input_pub = n.advertise<robonaldo::keyboard_input>("user_input", 1000);
	ros::Rate loop_rate(60);

	while (ros::ok() && windowOpen) {
		robonaldo::keyboard_input msg;

		getInput(msg);

		//ROS_INFO("Sending %u %u %u %u", msg.up, msg.down, msg.left, msg.right);

		user_input_pub.publish(msg);
    SDL_RenderPresent(renderer);
		ros::spinOnce();

		loop_rate.sleep();
	}

  if(ros::ok()){
    robonaldo::keyboard_input msg;
    msg.up = false;
    msg.down = false;
    msg.left = false;
    msg.right = false;
    user_input_pub.publish(msg);
    ros::spinOnce();
  }

  shutDownSDL();

	return 0;
}


