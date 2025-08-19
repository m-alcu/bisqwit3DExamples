#include "rasterize.hh"
#include <array>
#include <vector>
#include <map>

class Slope
{
    float begin, step;
public:
    Slope() {}
    Slope(float from, float to, int num_steps)
    {
        float inv_step = 1.f / num_steps;
        begin = from;                   // Begin here
        step  = (to - from) * inv_step; // Stepsize = (end-begin) / num_steps
    }
    float get() const { return begin; }
    void advance()    { begin += step; }
};

template<typename PlotFunc, typename T, std::size_t Size = std::tuple_size_v<T>>
void DrawSmoothColorPolygon(
    T p0,
    T p1,
    T p2,
    PlotFunc&& Plot)
{
    using SlopeData = std::array<Slope, Size-1>; // All input units except y

    RasterizeTriangle(
        &p0, &p1, &p2,
        // GetXY: Retrieve std::tuple<int,int> or std::array<int,2> from a PointType
        [&](const auto& p) { return std::tuple{ int(std::get<0>(p)), int(std::get<1>(p)) }; },
        // Slope generator
        [&](const T* from, const T* to, int num_steps)
        {
            SlopeData result;
            // Retrieve X coordinates for begin and end.
            result[0] = Slope( std::get<0>(*from), std::get<0>(*to), num_steps );
            // Take the rest of the props
            [&]<std::size_t...p>(std::index_sequence<p...>)
            {
                // Retrieve begin to end for each prop. The +2 skips x and y.
                ((result[p+1] = Slope( std::get<p+2>(*from), std::get<p+2>(*to), num_steps )), ...);
            }
            (std::make_index_sequence<Size-2>{});
            return result;
        },
        // Scanline function
        [&](int y, SlopeData& left, SlopeData& right)
        {
            int x = left[0].get(), endx = right[0].get();

            // Number of steps = number of pixels on this scanline = endx-x
            std::array<Slope, Size-2> props;
            for(unsigned p=0; p<Size-2; ++p)
            {
                props[p] = Slope( left[p+1].get(), right[p+1].get(), endx-x );
            }

            for(; x < endx; ++x) // Render each pixel
            {
                std::apply([&](auto... args) { Plot(x,y, args.get()...); }, props);

                // After each pixel, update the props by their step-sizes
                for(auto& prop: props) prop.advance();
            }

            // After the scanline is drawn, update the X coordinate and props on both sides
            for(auto& slope: left) slope.advance();
            for(auto& slope: right) slope.advance();
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
    std::vector< std::array<std::array<int,2>, 3> > triangles = CreateTriangleMesh(W,H,20);

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
        int color = 0x3B0103A5, b=19;
        for(auto& t: triangles)
        {
            color = ((color << b) | (unsigned(color) >> (32-b)));
            std::tuple p0 { t[0][0], t[0][1], (color&0xFF),((color>>8)&0xFF),((color>>16)&0xFF) };

            color = ((color << b) | (unsigned(color) >> (32-b)));
            std::tuple p1 { t[1][0], t[1][1], (color&0xFF),((color>>8)&0xFF),((color>>16)&0xFF) };

            color = ((color << b) | (unsigned(color) >> (32-b)));
            std::tuple p2 { t[2][0], t[2][1], (color&0xFF),((color>>8)&0xFF),((color>>16)&0xFF) };

            DrawSmoothColorPolygon(
                p0,p1,p2,
                [&](int x,int y, int b,int g,int r)
                {
                    pixels[y*W+x] = b+(g<<8)+(r<<16);
                });
        }
        SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
