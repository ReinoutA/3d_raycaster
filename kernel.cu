#include <SDL.h>
#include <iostream>
#include <vector>
#include <cuda_runtime.h>
#include <cmath>

#undef main


const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

SDL_Window* window;
SDL_Surface* screenSurface;

std::vector<std::vector<int>> world_map = {
        {1, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
};

float p_speler[] = {3, 3};
std::vector<double> r_speler = { -1.0 / std::sqrt(2), -1.0 / std::sqrt(2) };
std::vector<double> r_cameravlak = { 1.0 / std::sqrt(2), -1.0 / std::sqrt(2) };



void setupWindow() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
    }

    window = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == nullptr) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    }

    screenSurface = SDL_GetWindowSurface(window);
    SDL_FillRect(screenSurface, nullptr, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));
}

void drawSquare(int x, float length) {
    int squareX = x;
    int squareY = (SCREEN_HEIGHT - length) / 2;

    SDL_Rect squareRect = { squareX, squareY, 1, length };
    SDL_FillRect(screenSurface, &squareRect, SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0xFF));
}

std::vector<double> get_rStraal(int column) {
    double d_camera = 1.0;
    std::vector<double> r_straal = { 0, 0 };
    r_cameravlak = { r_speler[1], -r_speler[0] };
    r_straal[0] = (d_camera * r_speler[0]) + ((2 * (column / static_cast<double>(SCREEN_WIDTH)) - 1) * r_cameravlak[0]);
    r_straal[1] = (d_camera * r_speler[1]) + ((2 * (column / static_cast<double>(SCREEN_WIDTH)) - 1) * r_cameravlak[1]);
    double length = std::sqrt(r_straal[0] * r_straal[0] + r_straal[1] * r_straal[1]);
    r_straal[0] /= length;
    r_straal[1] /= length;
    return r_straal;
}

std::vector<float> raycast(std::vector<double> r_straal) {
    float delta_v = 0.0;
    float delta_h = 0.0;
    float d_horizontaal = 0.0;
    float d_verticaal = 0.0;

    if (r_straal[0] == 0) {
        delta_v = 1e30;
    }
    else {
        delta_v = 1 / std::abs(r_straal[0]);
    }
    if (r_straal[1] == 0) {
        delta_h = 1e30;
    }
    else {
        delta_h = 1 / std::abs(r_straal[1]);
    }

    if (r_straal[1] < 0) {
        d_horizontaal = (p_speler[1] - std::floor(p_speler[1])) * delta_h;
    }
    else {
        d_horizontaal = (1 - p_speler[1] + std::floor(p_speler[1])) * delta_h;
    }

    if (r_straal[0] < 0) {
        d_verticaal = (p_speler[0] - std::floor(p_speler[0])) * delta_v;
    }
    else {
        d_verticaal = (1 - p_speler[0] + std::floor(p_speler[0])) * delta_v;
    }

    std::vector<double> i_coordinaat = { 0.0, 0.0 };
    bool hit = false;
    int intersectie = 1;
    int mapX = 0;
    int mapY = 0;
    float d_muur = 100;

    while (!hit) {
        if (d_horizontaal <= d_verticaal) {
            i_coordinaat[0] = p_speler[0] + d_horizontaal * r_straal[0];
            i_coordinaat[1] = round(p_speler[1] + d_horizontaal * r_straal[1]);
            d_horizontaal += delta_h;
            intersectie = 1;

            // Calculate the cell we want to check
            if (r_straal[1] < 0) {
                mapY = static_cast<int>(std::floor(i_coordinaat[1] - 1));
                mapX = static_cast<int>(std::floor(i_coordinaat[0]));
            }
            else { // Check cell ABOVE
                mapY = static_cast<int>(std::floor(i_coordinaat[1]));
                mapX = static_cast<int>(std::floor(i_coordinaat[0]));
            }
        }
        else {
            i_coordinaat[0] = round(p_speler[0] + d_verticaal * r_straal[0]);
            i_coordinaat[1] = round(p_speler[1] + d_verticaal * r_straal[1]);
            d_verticaal += delta_v;
            intersectie = 0;

            // Calculate the cell we want to check
            if (r_straal[0] < 0) { // Check cell LEFT
                mapY = static_cast<int>(std::floor(i_coordinaat[1]));
                mapX = static_cast<int>(std::floor(i_coordinaat[0] - 1));
            }
            else { // Check cell RIGHT
                mapY = static_cast<int>(std::floor(i_coordinaat[1]));
                mapX = static_cast<int>(std::floor(i_coordinaat[0]));
            }
        }

        // Check if the point we're going to check is within the map
        if (mapX > (world_map.size() - 1) || mapX < 0 || mapY >(world_map[0].size() - 1) || mapY < 0) {
            d_muur = 100;
            intersectie = 0;
            hit = true;
        }
        else {
            if (world_map[mapY][mapX] > 0) {
                d_muur = std::sqrt(((i_coordinaat[0] - p_speler[0]) * (i_coordinaat[0] - p_speler[0])) + ((i_coordinaat[1] - p_speler[1]) * (i_coordinaat[1] - p_speler[1])));
                hit = true;
            }
        }
    }

    return { d_muur, static_cast<float>(intersectie) };
}

void renderColumn(int column, float d_muur) {
    float length = ((2 * SCREEN_WIDTH) / 3) / d_muur;
    drawSquare(column, length);
}



int main(int argc, char* args[]) {
    
    setupWindow();

    float i = 0;
    bool quit = false;
    SDL_Event e;

    // Main loop
    while (!quit) {
        SDL_FillRect(screenSurface, nullptr, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));

        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        for (int column = 0; column < SCREEN_WIDTH + 1; column++) {
            std::vector<double> r_straal = get_rStraal(column);
            std::vector<float> dMuur_Intersectie = raycast(r_straal);
            renderColumn(column, dMuur_Intersectie[0]);
        }

        SDL_UpdateWindowSurface(window);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
