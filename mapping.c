#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>
#include <SDL2/SDL.h>
#include <stdbool.h>
#include <time.h>

// Define constants
#define NUM_MICROPHONES 54
#define GRID_SIZE_X 200
#define GRID_SIZE_Y 200
#define PI 3.14159265359

// Structure to represent microphone positions
typedef struct {
    double x;
    double y;
} Point;

const Point mic_positions[NUM_MICROPHONES] = {
    {-30.635510051331634, -80.095999011857647},
    {-20.596792893808310, -80.095999011857647},
    {-10.558075736284987, -80.095999011857647},
    {-5.193585787616628, -80.095999011857647},
    {5.193585787616610, -80.095999011857647},
    {10.558075736284985, -80.095999011857647},
    {20.596792893808308, -80.095999011857647},
    {30.635510051331632, -80.095999011857647},
    {-50.193585787616620, -70.196443566095687},
    {-40.154868630093296, -70.196443566095687},
    {40.154868630093294, -70.196443566095687},
    {50.193585787616617, -70.196443566095687},
    {-50.712944366378282, -60.296888120333725},
    {50.712944366378279, -60.296888120333725},
    {-60.232302945139944, -50.397332674571765},
    {60.232302945139941, -50.397332674571765},
    {-60.751661523901605, -40.497777228809804},
    {60.751661523901605, -40.497777228809804},
    {-70.271020102663266, -30.598221783047843},
    {70.271020102663266, -30.598221783047843},
    {-70.790378681424929, -20.698666337285882},
    {70.790378681424929, -20.698666337285882},
    {-80.309737260186590, -10.799110891523922},
    {80.309737260186590, -10.799110891523922},
    {-80.829095838948251, -8.995554457619608},
    {80.829095838948251, -8.995554457619608},
    {-80.309737260186590, 0},
    {80.309737260186590, 0},
    {-80.829095838948251, 8.995554457619608},
    {80.829095838948251, 8.995554457619608},
    {-80.309737260186590, 10.799110891523922},
    {80.309737260186590, 10.799110891523922},
    {-70.790378681424929, 20.698666337285882},
    {70.790378681424929, 20.698666337285882},
    {-70.271020102663266, 30.598221783047843},
    {70.271020102663266, 30.598221783047843},
    {-60.751661523901605, 40.497777228809804},
    {60.751661523901605, 40.497777228809804},
    {-60.232302945139941, 50.397332674571765},
    {60.232302945139944, 50.397332674571765},
    {-50.712944366378279, 60.296888120333725},
    {50.712944366378282, 60.296888120333725},
    {-50.193585787616617, 70.196443566095687},
    {-40.154868630093294, 70.196443566095687},
    {40.154868630093296, 70.196443566095687},
    {50.193585787616620, 70.196443566095687},
    {-30.635510051331632, 80.095999011857647},
    {-20.596792893808308, 80.095999011857647},
    {-10.558075736284985, 80.095999011857647},
    {-5.193585787616610, 80.095999011857647},
    {5.193585787616628, 80.095999011857647},
    {10.558075736284987, 80.095999011857647},
    {20.596792893808310, 80.095999011857647},
    {30.635510051331634, 80.095999011857647},
};

// Base address of the AXI slave interface in the FPGA memory map
#define FPGA_AXI_BASE_ADDRESS 0x40000000

static unsigned int readMemoryAXI(unsigned int offset) {
    int fd;
    void *mapped_base;
    volatile unsigned int *axi_addr;

    // Open /dev/mem to access physical memory
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        perror("Error opening /dev/mem");
        return 0;
    }

    // Map the AXI slave interface to ARM memory space
    mapped_base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, FPGA_AXI_BASE_ADDRESS);
    if (mapped_base == MAP_FAILED) {
        perror("Error mapping memory");
        close(fd);
        return 0;
    }

    // Set the pointer to the base address of the AXI slave interface
    axi_addr = (volatile unsigned int *)mapped_base;

    // Read data from the FPGA fabric
    unsigned int data = axi_addr[0]+offset;

    // Unmap memory
    munmap(mapped_base, 0x1000);
    close(fd);

    return data;
}

// Function to estimate angles based on cross-correlation
static void estimate_angles(double signals[NUM_MICROPHONES], double *angles) {
    // Iterate over pairs of microphones
    for (int i = 0; i < NUM_MICROPHONES; i++) {
        // Initialize variables to keep track of max correlation and corresponding angle
        double max_corr = 0.0;
        double angle_with_max_corr = 0.0;

        for (int j = 0; j < NUM_MICROPHONES; j++) {
            // Calculate cross-correlation
            double corr = 0.0;
            corr = signals[i] * signals[j];

            // Update maximum correlation and corresponding angle
            if (corr > max_corr) {
                max_corr = corr;
                // Calculate the angle between the microphones
                double dx = mic_positions[j].x - mic_positions[i].x;
                double dy = mic_positions[j].y - mic_positions[i].y;
                angle_with_max_corr = atan2(dy, dx);
            }
        }

        // Convert angle to degrees
        //angle_with_max_corr *= 180.0 / M_PI;

        // Store the estimated angle for the current microphone
        angles[i] = angle_with_max_corr;
    }
}
// Function to map angle to pixel value in the range [0, 255]
static uint8_t map_angle_to_pixel(double angle) {
    // Map angle to pixel value in the range [0, 255]
    return 255 - (uint8_t)((angle / (2 * PI)) * 255);
}

static int generate_image(uint8_t heat_map[GRID_SIZE_X][GRID_SIZE_Y], double angles[NUM_MICROPHONES]) {
    // Iterate over grid points
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            // Initialize weighted sum and total weight
            double weighted_sum = 0.0;
            double total_weight = 0.0;

            // Iterate over microphones
            for (int k = 0; k < NUM_MICROPHONES; k++) {
                // Calculate distance between grid point and microphone
                double dx = i - mic_positions[k].x - (GRID_SIZE_X / 2);
                double dy = j - mic_positions[k].y - (GRID_SIZE_Y / 2);
                double distance = sqrt(dx * dx + dy * dy); // Euclidean distance

                // Calculate weight
                double weight = 1.0 / distance;

                // Update weighted sum and total weight
                weighted_sum += angles[k] * weight;
                total_weight += weight;
            }

            // Normalize weights
            if (total_weight != 0.0) {
                weighted_sum /= total_weight;
            }

            // Map interpolated angle to pixel value
            heat_map[i][j] = map_angle_to_pixel(weighted_sum);
        }
    }
    return 0;
}

/*static int generate_image(uint8_t heat_map[GRID_SIZE_X][GRID_SIZE_Y], double time_delays[NUM_MICROPHONES]) {
    	// Direction of Arrival (DOA) Estimation
    	double angles[NUM_MICROPHONES];
        estimate_angles(time_delays, angles);

    	// Interpolate estimated angles onto the grid and map to pixel values
    	for (int i = 0; i < GRID_SIZE_X; i++) {
        	for (int j = 0; j < GRID_SIZE_Y; j++) {
            		// Calculate distance of grid point from each microphone and sum angles weighted by inverse distance
            		double weighted_sum = 0.0;
            		double total_weight = 0.0;
            		for (int k = 0; k < NUM_MICROPHONES; k++) {
                		double distance_squared = pow((i - mic_positions[k].x - (GRID_SIZE_X/2)), 2) + pow((j - mic_positions[k].y - (GRID_SIZE_Y/2)), 2);
                		double weight = 1.0 / distance_squared;
                		weighted_sum += angles[k] * weight;
                		total_weight += weight;
            		}
            		double interpolated_angle = weighted_sum / total_weight;
            		heat_map[i][j] = map_angle_to_pixel(interpolated_angle);
        	}
    	}
	return 0;
}*/

int main(int argc, char** argv) {
    int running = 1;
    double time_delays[NUM_MICROPHONES];
    // Heat Map Generation
    // Create a grid representing the space surrounding the microphone array
    uint8_t heat_map[GRID_SIZE_X][GRID_SIZE_Y];
    srand(time(NULL));
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
	    SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
	    return 1;
    }

    // Create a window
    SDL_Window* window = SDL_CreateWindow("Ultrasound Image", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 800, SDL_WINDOW_SHOWN | SDL_WINDOW_MAXIMIZED);
    if (window == NULL) {
	    SDL_Log("Unable to create window: %s", SDL_GetError());
	    return 1;
    }

    // Create a renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
	    SDL_Log("Unable to create renderer: %s", SDL_GetError());
	    return 1;
    }
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_Rect destination = { 0, 0, 800, 800 }; // Destination rectangle for the texture

    // Set colours
    SDL_Color colors[256];
    for (int i = 0; i < 256; i++) {
	    colors[i].r = colors[i].g = colors[i].b = i;
    }

    while (running) {
#if 1
	for (int i = 0; i < NUM_MICROPHONES; i++) {
		time_delays[i] = rand() % 10 + 1; // Randomly generated
	}

	generate_image(heat_map, time_delays);

	// Create a surface and fill it with a color
	SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormatFrom(heat_map, GRID_SIZE_X, GRID_SIZE_Y, 8, GRID_SIZE_X, SDL_PIXELFORMAT_INDEX8); // Create a surface
	if (surface == NULL) {
		SDL_Log("Unable to create surface: %s", SDL_GetError());
		return 1;
	}
	SDL_SetPaletteColors(surface->format->palette, colors, 0, 256);

	// Create a texture from the surface
	SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
	if (texture == NULL) {
		SDL_Log("Unable to create texture from surface: %s", SDL_GetError());
		return 1;
	}

	// Clear the renderer
	SDL_RenderClear(renderer);

	// Draw the texture (square)
	SDL_RenderCopy(renderer, texture, NULL, &destination);

	// Present the renderer
	SDL_RenderPresent(renderer);

	// Wait until the user closes the program
	SDL_Event event;
	if (SDL_PollEvent(&event)) {
        	if (event.type == SDL_QUIT) {
        		running = false;
        	}
	}

	// Cleanup
	SDL_FreeSurface(surface);
	SDL_DestroyTexture(texture);
	SDL_Delay(600);
#else
	SDL_HideWindow(window);
	for (int i = 0; i < NUM_MICROPHONES; i++) {
		time_delays[i] = readMemoryAXI(i);
	}

	generate_image(heat_map, time_delays);

	// Save Heat Map to Raw Image File
	FILE *image_file = fopen("heat_map.raw", "wb");
	if (!image_file) {
        	fprintf(stderr, "Error: Unable to open output file.\n");
        	return 1;
	}

	// Write raw image data
	for (int i = 0; i < GRID_SIZE_X; i++) {
        	fwrite(&heat_map[i], sizeof(uint8_t), GRID_SIZE_Y, image_file);
	}
	
	fclose(image_file);
#endif
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
