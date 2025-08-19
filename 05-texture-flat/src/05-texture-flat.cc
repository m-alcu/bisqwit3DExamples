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
void DrawTexturedPolygon(
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
            // Number of steps = number of scanlines
            // Retrieve X coordinates for begin and end
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

#include <SDL3/SDL.h>

int main()
{
    const int W = 424, H = 240;
    // Create a screen.
    SDL_Window* window = SDL_CreateWindow("Chip8", W*4,H*4, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W,H);
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);

    const int txW = 256, txH = 256;
    unsigned bitmap[txW*txH];

    for(unsigned y=0; y<txH; ++y)
        for(unsigned x=0; x<txW; ++x)
        {
            int l = (0x1FF >> std::min({x,y,txW-1-x,txH-1-y,31u}));
            int d = std::min(50,std::max<int>(0,255-50*std::pow(std::hypot(x/float(txW/2)-1.f, y/float(txH/2)-1.f)*4,2.f)));
            int r = (~x & ~y)&255, g = (x & ~y)&255, b = (~x & y)&255;
            bitmap[y*txW+x] = std::min(std::max(r-d,l),255)*65536 + std::min(std::max(g-d,l),255)*256 + std::min(std::max(b-d,l),255);
        }

    // Create a rectangle comprised of two triangles
    std::vector< std::array<std::array<float,4>, 3> > triangles
    {
        { std::array{ 120.f,40.f, 0.f,0.f }, { 300,40, 0,txH }, { 300,200, txW,txH } },
        { std::array{ 120.f,40.f, 0.f,0.f },                    { 300,200, txW,txH }, { 120,200, txW,0 } }
    };

    #include "mousemanip-part1.hh"

    for(bool interrupted=false; !interrupted; )
    {
        // Process events.
        SDL_Event ev;
        while(SDL_PollEvent(&ev))

            switch(ev.type)
            {
                case SDL_EVENT_QUIT: interrupted = true; break;
                #include "mousemanip-part2.hh"
            }

        // Render graphics
        Uint32 pixels[W*H]={};
        for(auto& t: triangles)
        {
            DrawTexturedPolygon(
                t[0], t[1], t[2],
                [&](int x,int y, int u,int v)
                {
                    if(y>=0 && x>=0 && y<H && x<W)
                    {
                        pixels[y*W+x] = bitmap[(u%txW)*txW + v%txH];
                    }
                });
        }
        SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
