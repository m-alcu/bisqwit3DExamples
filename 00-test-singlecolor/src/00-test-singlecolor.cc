#include "rasterize.hh"
#include <array>
#include <vector>
#include <map>

template<typename PlotFunc>
void DrawFilledSingleColorPolygon(
    std::array<int,2> p0,
    std::array<int,2> p1,
    std::array<int,2> p2,
    PlotFunc&& Plot)
{
    using SlopeData = std::pair<float/*begin*/, float/*step*/>;

    RasterizeTriangle(
        &p0, &p1, &p2,
        // GetXY: Retrieve std::tuple<int,int> or std::array<int,2> from a PointType
        [&](const auto& p) -> const std::array<int,2>& { return p; },
        // Slope generator
        [&](const std::array<int,2>* from, const std::array<int,2>* to, int num_steps)
        {
            // Retrieve X coordinates for begin and end
            int begin = (*from)[0], end = (*to)[0];
            // Number of steps = number of scanlines
            float inv_step = 1.f / num_steps;
            return SlopeData{ begin,                      // Begin here
                              (end - begin) * inv_step }; // Stepsize = (end-begin) / num_steps
        },
        // Scanline function
        [&](int y, SlopeData& left, SlopeData& right)
        {
            int x = left.first, endx = right.first;
            for(; x < endx; ++x) // Render each pixel
            {
                Plot(x, y);
            }

            // After the scanline is drawn, update the X coordinate on both sides
            left.first += left.second;
            right.first += right.second;
        });
}


#include "mesh.hh"

#include <SDL3/SDL.h>

int main()
{
    const int W = 424, H = 240;
    // Create a screen.
    SDL_Window* window = SDL_CreateWindow("Chip8", W*4,H*4, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);

    // Create a mesh of triangles covering the entire screen
    std::vector< std::array<std::array<int,2>, 3> > triangles = CreateTriangleMesh(W,H,100);

    for(bool interrupted=false; !interrupted; )
    {
        // Process events.
        SDL_Event ev;
        while(SDL_PollEvent(&ev))

            switch(ev.type)
            {
                case SDL_EVENT_QUIT: interrupted = true; break;
            }
        // Render graphics
        Uint32 pixels[W*H]={};
        unsigned color = 0x3B0103A5, blank = 0xFFFFFF, duplicate = 0xFFAA55;
        for(auto& p: pixels) p = blank;
        for(auto& t: triangles)
        {
            color = ((color << 1) | (color >> (32-1)));
            DrawFilledSingleColorPolygon(
                t[0], t[1], t[2],
                [&](int x,int y)
                {
                    if(pixels[y*W+x] != blank)
                        pixels[y*W+x] = duplicate;
                    else
                        pixels[y*W+x] = (color&0xFFFFFF);
                });
        }
        SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
