#include <SDL2/SDL.h>
#include "ros/ros.h"
#include "robonaldo_msgs/keyboard_input.h"
#include <sstream>
#include <termios.h>
#include <stdio.h>
#include <SDL2/SDL.h>

const int windowWidth = 640;
const int windowHeight = 480;
const int keySize = 50;
const int keySpacing = 10;
const int heightOffset = -20;

// Define the squares for the keys based on the constants above.
const SDL_Rect upArrow = { .x = (windowWidth / 2) - (keySize / 2), .y = (windowHeight / 2) - (keySize + heightOffset + keySpacing + (keySize / 2)), .w = keySize, .h = keySize };
const SDL_Rect downArrow = { .x = (windowWidth / 2) - (keySize / 2), .y = (windowHeight / 2) - (heightOffset + (keySize / 2)), .w = keySize, .h = keySize };
const SDL_Rect leftArrow = { .x = (windowWidth / 2) - (keySize / 2) - (keySize + keySpacing), .y = (windowHeight / 2) - (heightOffset + (keySize / 2)), .w = keySize, .h = keySize };
const SDL_Rect rightArrow = { .x = (windowWidth / 2) - (keySize / 2) + (keySize + keySpacing), .y = (windowHeight / 2) - (heightOffset + (keySize / 2)), .w = keySize, .h = keySize };

SDL_Window* window;
SDL_Renderer* renderer;
bool windowOpen = true;

void initSDL()
{

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0)
  {
    fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
  }

  window = SDL_CreateWindow("Robot Control", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, windowWidth, windowHeight, 0);

  if (window == nullptr)
  {
    fprintf(stderr, "could not create window: %s\n", SDL_GetError());
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
}

void shutDownSDL()
{
  SDL_DestroyRenderer(renderer);
  renderer = nullptr;

  SDL_DestroyWindow(window);
  window = nullptr;

  SDL_Quit();
}

/* Get user input and set msg accordingly, using WASD */
void getInput(robonaldo_msgs::keyboard_input& msg)
{

  SDL_Event event;
  while (SDL_PollEvent(&event))
  {

    if (event.type == SDL_QUIT)
    {
      windowOpen = false;
    }

  }

  const Uint8* state = SDL_GetKeyboardState(nullptr);

  // Check if excape was hit.
  if (state[SDL_SCANCODE_ESCAPE])
  {
    windowOpen = false;
  }

  msg.up = state[SDL_SCANCODE_W] | state[SDL_SCANCODE_UP];
  msg.left = state[SDL_SCANCODE_A] | state[SDL_SCANCODE_LEFT];
  msg.down = state[SDL_SCANCODE_S] | state[SDL_SCANCODE_DOWN];
  msg.right = state[SDL_SCANCODE_D] | state[SDL_SCANCODE_RIGHT];

}


int main(int argc, char **argv)
{

  initSDL();

  ros::init(argc, argv, "desktop");
  ros::NodeHandle n;
  ros::Publisher user_input_pub = n.advertise<robonaldo_msgs::keyboard_input>("user_input", 1000);
  ros::Rate loop_rate(60);

  while (ros::ok() && windowOpen)
  {
    robonaldo_msgs::keyboard_input msg;

    getInput(msg);

    // Set screen to black
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Draw up
    Uint8 g = msg.up ? 0 : 255;
    Uint8 b = msg.up ? 0 : 255;
    SDL_SetRenderDrawColor(renderer, 255, g, b, 255);
    SDL_RenderFillRect(renderer, &upArrow);

    // Draw down
    g = msg.down ? 0 : 255;
    b = msg.down ? 0 : 255;
    SDL_SetRenderDrawColor(renderer, 255, g, b, 255);
    SDL_RenderFillRect(renderer, &downArrow);

    // Draw left
    g = msg.left ? 0 : 255;
    b = msg.left ? 0 : 255;
    SDL_SetRenderDrawColor(renderer, 255, g, b, 255);
    SDL_RenderFillRect(renderer, &leftArrow);

    // Draw right
    g = msg.right ? 0 : 255;
    b = msg.right ? 0 : 255;
    SDL_SetRenderDrawColor(renderer, 255, g, b, 255);
    SDL_RenderFillRect(renderer, &rightArrow);

    SDL_RenderPresent(renderer);

    //ROS_INFO("Sending %u %u %u %u", msg.up, msg.down, msg.left, msg.right);
    user_input_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  if (ros::ok())
  {
    robonaldo_msgs::keyboard_input msg;
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


