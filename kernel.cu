/*
#include <SDL.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <string>
#include <SDL_image.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#undef main


const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;
const int IMG_SIZE = 512;
SDL_Window* window;
SDL_Surface* screenSurface;
Uint32* img;

int frame_rate = 0;
double elapsed_time = 0.0;
int world_map[15][15];
std::vector<double> fps_history;
int world_map_len = 15;
float p_speler[] = { 3, 3 };
float r_straal[] = { 1.0 / std::sqrt(2), -1.0 / std::sqrt(2) };
float r_speler[] = { 1 / sqrt(2) , -1 / sqrt(2) };
float r_cameravlak[] = { -1 / sqrt(2) , -1 / sqrt(2) };

void initializeWorldMap() {
    FILE* file = fopen("world_map.txt", "r");
    if (file == NULL) {
        printf("Kon het bestand niet openen.");
        return;
    }

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            fscanf(file, "%d", &world_map[i][j]);
        }
    }
    fclose(file);
}

void setupWindow() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
    }

    window = SDL_CreateWindow("SDL Raycaster", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == nullptr) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    }

    screenSurface = SDL_GetWindowSurface(window);
    //SDL_FillRect(screenSurface, nullptr, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));
}

Uint32* loadImage(const char* file) {
    SDL_Surface* img = IMG_Load(file);
    img = SDL_ConvertSurfaceFormat(img, SDL_PIXELFORMAT_RGBA32, 0);
    return static_cast<Uint32*>(img->pixels);
}

void getImage(const char* file) {
    img = loadImage(file);
    if (img == NULL) {
        std::cout << "img was nullptr";
    }
}

float get_r_straal_x(int column) {
    return r_speler[0] + (2 * (column / static_cast<double>(SCREEN_WIDTH)) - 1) * r_cameravlak[0];
}

float get_r_straal_y(int column) {
    return r_speler[1] + (2 * (column / static_cast<double>(SCREEN_WIDTH)) - 1) * r_cameravlak[1];
}

void renderImageKolom(int kolom, float d_muur, int intersectie, float i_x, float i_y) {
    // Bereken de hoogte van het te renderen deel van de afbeelding op basis van de muurafstand
    int img_height_screen = static_cast<int>((SCREEN_HEIGHT / d_muur));
    float img_scale = (float)IMG_SIZE / img_height_screen;
    int render_y_begin = (SCREEN_HEIGHT - img_height_screen) / 2;
    float img_y;
    if (render_y_begin >= 0) {
        img_y = 0.0;
    }
    else {
        img_y = -render_y_begin;
    }

    // Bepaal de x-positie op de afbeelding op basis van de intersectie
    int img_x;
    if (intersectie == 1)
        img_x = (int)(IMG_SIZE * (i_x - std::floor(i_x)));
    else
        img_x = (int)(IMG_SIZE * (i_y - std::floor(i_y)));

    // Render elke pixel van de afbeelding op de kolom op het scherm
    for (int screen_y = std::max(render_y_begin, 0); screen_y < std::min(render_y_begin + img_height_screen, SCREEN_HEIGHT); ++screen_y, ++img_y) {
        // Bereken de index van de pixel in de afbeelding
        int img_idx = (int)(img_y * img_scale) * IMG_SIZE + img_x;
        // Bereken de index op het scherm
        int screen_idx = kolom + SCREEN_WIDTH * screen_y;
        // Kopieer de pixelwaarde van de afbeelding naar het scherm
        ((Uint32*)screenSurface->pixels)[screen_idx] = img[img_idx];
    }
}

void raycast(int column) {
    float delta_v = 0.0;
    float delta_h = 0.0;
    float d_horizontaal = 0.0;
    float d_verticaal = 0.0;

    // Bereken de coördinaten van r_cameravlak
    float r_straal_x = get_r_straal_x(column);
    float r_straal_y = get_r_straal_y(column);

    delta_v = (r_straal_x == 0) ? 1e30 : 1 / std::abs(r_straal_x);
    delta_h = (r_straal_y == 0) ? 1e30 : 1 / std::abs(r_straal_y);

    // Bereken d_horizontaal en d_verticaal
    d_horizontaal = (r_straal_y < 0) ? (p_speler[1] - std::floor(p_speler[1])) * delta_h : (1 - p_speler[1] + std::floor(p_speler[1])) * delta_h;
    d_verticaal = (r_straal_x < 0) ? (p_speler[0] - std::floor(p_speler[0])) * delta_v : (1 - p_speler[0] + std::floor(p_speler[0])) * delta_v;

    bool hit = false;
    int intersectie = 1;
    int mapX = 0;
    int mapY = 0;
    float d_muur = 100;
    float i_x, i_y;

    while (!hit) {
        if (d_verticaal >= d_horizontaal) {
            i_x = r_straal_x * d_horizontaal + p_speler[0];
            i_y = r_straal_y * d_horizontaal + p_speler[1];

            if (mapX > (world_map_len - 1) || mapX < 0 || mapY >(world_map_len - 1) || mapY < 0) {
                d_muur = 100;
                hit = true;
            }
            else {
                if (world_map[static_cast<int>(std::floor(i_x))][static_cast<int>(std::round(i_y) + ((r_straal_y < 0) ? -1.0 : 0.0))] > 0) {
                    d_muur = d_horizontaal * (r_straal_x * r_speler[0] + r_straal_y * r_speler[1]);
                    hit = true;
                }
            }
            mapX++;
            d_horizontaal += delta_h;
        }
        else {
            i_x = r_straal_x * d_verticaal + p_speler[0];
            i_y = r_straal_y * d_verticaal + p_speler[1];

            if (mapX > (world_map_len - 1) || mapX < 0 || mapY >(world_map_len - 1) || mapY < 0) {
                d_muur = 100;
                intersectie = 0;
                hit = true;
            }
            else {
                if (world_map[static_cast<int>(std::round(i_x) + ((r_straal_x < 0) ? -1.0 : 0.0))][static_cast<int>(std::floor(i_y))] > 0) {
                    d_muur = d_verticaal * (r_straal_x * r_speler[0] + r_straal_y * r_speler[1]);
                    intersectie = 0;
                    hit = true;
                }
            }
            mapY++;
            d_verticaal += delta_v;
        }

    }
    renderImageKolom(column, d_muur, intersectie, i_x, i_y);
}

// Functie om spelerbeweging te verwerken
void handleMovement(SDL_Event& e) {
    const Uint8* currentKeyStates = SDL_GetKeyboardState(nullptr);
    double speed = 0.01;
    double angularSpeed = 0.02; // Angulaire snelheid (rotatie per frame)

    // Check vooruit
    if (currentKeyStates[SDL_SCANCODE_W]) {
        p_speler[0] += r_speler[0] * speed;
        p_speler[1] += r_speler[1] * speed;
    }
    // Check achteruit
    if (currentKeyStates[SDL_SCANCODE_S]) {
        p_speler[0] -= r_speler[0] * speed;
        p_speler[1] -= r_speler[1] * speed;
    }
    // Check links
    if (currentKeyStates[SDL_SCANCODE_A]) {
        // Beweeg de speler zijwaarts naar links
        p_speler[0] -= r_speler[1] * speed;
        p_speler[1] += r_speler[0] * speed;
    }
    // Check rechts
    if (currentKeyStates[SDL_SCANCODE_D]) {
        // Beweeg de speler zijwaarts naar rechts
        p_speler[0] += r_speler[1] * speed;
        p_speler[1] -= r_speler[0] * speed;
    }

    // Check links (rotatie tegen de klok in)
    if (currentKeyStates[SDL_SCANCODE_F]) {
        double cosA = std::cos(-angularSpeed);
        double sinA = std::sin(-angularSpeed);
        double x = r_speler[0];
        r_speler[0] = cosA * x - sinA * r_speler[1];
        r_speler[1] = sinA * x + cosA * r_speler[1];

        // Bereken nieuwe richting voor r_cameravlak
        float new_r_cameravlak_x = cosA * r_cameravlak[0] - sinA * r_cameravlak[1];
        float new_r_cameravlak_y = sinA * r_cameravlak[0] + cosA * r_cameravlak[1];
        r_cameravlak[0] = new_r_cameravlak_x;
        r_cameravlak[1] = new_r_cameravlak_y;
    }
    // Check rechts (rotatie met de klok mee)
    if (currentKeyStates[SDL_SCANCODE_E]) {
        double cosA = std::cos(angularSpeed);
        double sinA = std::sin(angularSpeed);
        double x = r_speler[0];
        r_speler[0] = cosA * x - sinA * r_speler[1];
        r_speler[1] = sinA * x + cosA * r_speler[1];

        // Bereken nieuwe richting voor r_cameravlak
        float new_r_cameravlak_x = cosA * r_cameravlak[0] - sinA * r_cameravlak[1];
        float new_r_cameravlak_y = sinA * r_cameravlak[0] + cosA * r_cameravlak[1];
        r_cameravlak[0] = new_r_cameravlak_x;
        r_cameravlak[1] = new_r_cameravlak_y;
    }
}


void calculateAndSetFPSTitle(double deltaTime) {
    frame_rate++;
    elapsed_time += deltaTime;

    if (elapsed_time >= 1.0) {
        float frame_rate_per_sec = static_cast<double>(frame_rate) / elapsed_time;
        fps_history.push_back(frame_rate_per_sec); // Voeg de huidige fps toe aan de geschiedenis
        if (fps_history.size() > 1) { // Houd slechts de fps van de afgelopen 10 seconden bij
            fps_history.erase(fps_history.begin());
        }

        // Bereken gemiddelde FPS van de afgelopen 10 seconden
        double sum = 0.0;
        for (double fps : fps_history) {
            sum += fps;
        }
        double average_fps = sum / fps_history.size();

        // Open het bestand in append-modus en schrijf het gemiddelde FPS
        std::ofstream outfile("gemiddelde_fps.txt", std::ios::app);
        if (outfile.is_open()) {
            outfile << average_fps << std::endl;
            outfile.close();
        }
        else {
            std::cerr << "Kon gemiddelde_fps.txt niet openen voor schrijven!" << std::endl;
        }

        // Stel de venstertitel in met de huidige FPS
        std::stringstream stream;
        stream << "Frame Rate: " << static_cast<int>(frame_rate_per_sec);
        SDL_SetWindowTitle(window, stream.str().c_str());

        // Reset frame_rate en elapsed_time voor de volgende meting
        frame_rate = 0;
        elapsed_time = 0.0;
    }
}

int main(int argc, char* args[]) {

    setupWindow();

    getImage("muur.png");
    initializeWorldMap();
    bool quit = false;
    SDL_Event e;

    auto start_time = std::chrono::high_resolution_clock::now();
    while (!quit) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        handleMovement(e);

        for (int pixel_idx = 0; pixel_idx < SCREEN_WIDTH * SCREEN_HEIGHT; ++pixel_idx) {
            ((Uint32*)screenSurface->pixels)[pixel_idx] = 0xFFFFFFFF;
        }

        for (int column = 0; column <= SCREEN_WIDTH - 1; column++) {
            raycast(column);
        }

        SDL_UpdateWindowSurface(window);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> deltaTime = end_time - start_time;
        start_time = end_time;

        calculateAndSetFPSTitle(deltaTime.count());
    }
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
*/