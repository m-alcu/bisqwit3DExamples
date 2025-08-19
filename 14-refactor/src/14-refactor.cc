#include "polygon_clip.hh"
#include "polygon_draw.hh"
#include "math.hh"
#include "view.hh"

#include <map>
#include <array>
#include <ranges>
#include <random>
#include <vector>
#include <cstdint>

#include <SDL3/SDL.h>

struct Polygon
{
    // Vertex list
    std::size_t first, size; // Refers to a separate table of vertices
    unsigned    ubase, vbase, usize, vsize;
    unsigned    flags;
};

void Render(const auto& vertices, const auto& polys, auto& view, const auto& frustum,
            auto&& tform, auto&& plot)
{
    for(const auto& poly: polys)
    {
        // Collect the corners of the polygon. Translate and rotate them.
        std::vector<std::invoke_result_t<decltype(tform), decltype(vertices[0])>> points;
        for(unsigned n=0; n<poly.size; ++n)
            points.emplace_back(tform(vertices[poly.first + n]));

        // Optional: Discard polygons that are not facing the camera (back-face culling).
        // This is calculated by           ((p1-p0) × (p2-p0)) · p0
        // which could be optimized as...: ((p1 × p2)) · p0
        if(Dot(CrossProduct(points[1], points[2]), points[0]) < -1e-5f) continue;

        // Clip the polygon against the frustum while it’s still 3D.
        for(const Plane& p: frustum) ClipPolygon(p, points);

        // If the polygon is no longer a surface, don’t try to render it.
        if(points.size() < 3) continue;

        // Perspective-project whatever points remain. Now it’s 2D, but with a copy of the original Z coordinate.
        for(auto& p: points)
            p = std::apply([&](auto,auto, auto&&... rest) // Replace the original x & y with perspective-corrected ones
            {
                return AsArray(view.PerspectiveProject(p), rest...);
            }, p);

        DrawPolygon(points, view.Draw([&](auto&&... args) { return plot(poly, std::forward<decltype(args)>(args)...); }));
    }
}

auto CreateLevelMap()
{
    std::vector<std::array<float,8>> points;
    std::vector<Polygon> poly;

    auto addcuboid = [&](std::array<float,2> x, std::array<float,2> z, std::array<float,2> y,
                         std::array<float,3> c, std::array<float,3> u, std::array<float,3> v)
    {
        auto ext = [](auto m,unsigned n,unsigned b=1) { return (m >> (n*b)) & ~(~0u << b); }; // extracts bits
        // Generates: For six vertices, coordinate(xyz), texture coord(uv), and color(rgb).
        std::array p{&x[0],&y[0],&z[0], &u[0],&v[0], &c[0],&c[0],&c[0]};
        // X(4 bits), Y(4 bits), Z(4 bits), U(4 bits), V(4 bits)
        for(unsigned m: std::array{0x60339,0x9F339,0x36039,0xC6F39,0x06C39,0xF6339}) // bottom, top, four sides
            if(std::uint64_t s = 0b111'11'000 * (~0llu/255))
            {
                poly.emplace_back( Polygon{points.size(),4, 0,0, 256,256, 0} );
                for(unsigned q = 4; q-- > 0; )
                    points.emplace_back([&]<std::size_t...n>(std::index_sequence<n...>)
                        {
                            return std::array{ p[n][ext(m, ext(044412345u, n, 3)*4 - (1+q)) << ext(s, n+q*8)] ... };
                            // 44412345 = nibble indexes in "m" for each of 8 values
                        }(std::make_index_sequence<8>{}));
            }
    };
    addcuboid({-10,10}, {-10,10}, {-10,10}, {1,1,1}, {0,256,256},  {0,256,256});
    return std::pair(std::move(points), std::move(poly));
}

int main()
{
    const int W = 848, H = 480;
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
            int l = std::min(0x1FF >> std::min({x,y,txW-1-x,txH-1-y,31u}), 255);
            int d = std::clamp<int>(255-50*std::pow(std::hypot(x/float(txW/2)-1.f, y/float(txH/2)-1.f)*4,2.f),0,50);
            bitmap[y*txW+x] = Sum(std::tuple{65536,256,1} * std::tuple{std::clamp(int((~x & ~y)&255) - d,l,255),
                                                                       std::clamp(int(( x & ~y)&255) - d,l,255),
                                                                       std::clamp(int((~x &  y)&255) - d,l,255)});
        }

    auto [vertices,polys] = CreateLevelMap();

    auto Plot = [&](auto& poly, auto,auto,auto, unsigned u,unsigned v, auto&&...)
    {
        unsigned texel = bitmap[ (poly.vbase + (v - poly.vbase) % poly.vsize) * txW
                               + (poly.ubase + (u - poly.ubase) % poly.usize) ];
        return texel;
    };

    View view(W,H, 120.f/* degrees */);
    auto frustum = view.MakeFrustum();

    std::tuple r{0.f, 0.f, .2f};             // Rotation momentum vector (nonzero indicates view is still rotating)
    std::tuple m{0.f, 0.f, 0.f};             // Movement momentum vector (nonzero indicates camera is still moving)
    std::tuple l{6.9f,0.146f,-21.f};         // Camera location (X,Y,Z) coordinate
    float aa=0.094,ab=-0.11,ac=-0.021,ad=1;  // View rotation quaternion
    float tform[16]{};                       // View rotation matrix (calculated from the quaternion)

    // Main loop
    for(std::map<int,bool> keys; !keys[SDLK_ESCAPE]; )
    {
        // Process events.
        for(SDL_Event ev; SDL_PollEvent(&ev); )
            switch(ev.type)
            {
                case SDL_EVENT_QUIT: keys[SDLK_ESCAPE] = true; break;
                case SDL_EVENT_KEY_DOWN: keys[ev.key.key] = true; break;
                case SDL_EVENT_KEY_UP:   keys[ev.key.key] = false; break;
            }
        // The input scheme is the same as in Descent, the game by Parallax Interactive.
        // Mouse input is not handled for now.
        bool up    = keys[SDLK_UP]   || keys[SDLK_KP_8];
        bool down  = keys[SDLK_DOWN] || keys[SDLK_KP_2],     alt   = keys[SDLK_LALT]|| keys[SDLK_RALT];
        bool left  = keys[SDLK_LEFT] || keys[SDLK_KP_4],     rleft = keys[SDLK_Q]   || keys[SDLK_KP_7];
        bool right = keys[SDLK_RIGHT]|| keys[SDLK_KP_6],     rright= keys[SDLK_E]   || keys[SDLK_KP_9];
        bool fwd   = keys[SDLK_A], sup   = keys[SDLK_KP_MINUS], sleft = keys[SDLK_KP_1];
        bool back  = keys[SDLK_Z], sdown = keys[SDLK_KP_PLUS],  sright= keys[SDLK_KP_3];
        // Change the rotation momentum vector (r) with hysteresis: newvalue = oldvalue*(1-eagerness) + input*eagerness
        r = (r * .9f) + std::tuple{0.f+(up     - down) * !alt,
                                   0.f+(right  - left) * !alt,
                                   0.f+(rright - rleft)} * .1f;
        if(float rlen = Length(r); rlen > 1e-3f) // Still rotating?
        {
            // Create rotation change quaternion (q) relative to the direction that the camera looks
            // by multiplying the rotation momentum vector (r) with the current rotation matrix.
            float theta = rlen*.03f, c = std::cos(theta*.5f), s = std::sin(theta*.5f)/rlen;
            std::tuple q{ c,
                          s * Dot(r, {tform[0],tform[1],tform[2]}),
                          s * Dot(r, {tform[4],tform[5],tform[6]}),
                          s * Dot(r, {tform[8],tform[9],tform[10]}) };
            // Update the rotation quaternion (a) by multiplying it by the rotation change quaternion (q):
            std::tie(aa,ab,ac,ad) = Normalized(std::tuple{ Dot(q, {aa,-ab,-ac,-ad}),
                                                           Dot(q, {ab, aa,-ad, ac}),
                                                           Dot(q, {ac, ad, aa,-ab}),
                                                           Dot(q, {ad,-ac, ab, aa})});
            // Convert the rotation quaternion (a) into rotation matrix using formula from Wikipedia:
            tform[0] = 1-2*(ac*ac+ad*ad); tform[1] =   2*(ab*ac+aa*ad); tform[2] =   2*(ab*ad-aa*ac);
            tform[4] =   2*(ab*ac-aa*ad); tform[5] = 1-2*(ab*ab+ad*ad); tform[6] =   2*(ac*ad+aa*ab);
            tform[8] =   2*(ab*ad+aa*ac); tform[9] =   2*(ad*ac-aa*ab); tform[10]= 1-2*(ab*ab+ac*ac);
        }
        // Camera movement vector
        std::array M{ 0.f+((sleft || (alt && left)) - (sright || (alt && right))),
                      0.f+((sdown || (alt && down)) - (sup    || (alt && up))),
                      0.f+(fwd - back) };
        float mlen = 2*Length(M); if(mlen < 1e-3f) mlen = 1;
        // Multiply with rotation matrix (tform) and apply with hysteresis to movement momentum vector (m).
        m = (m * .9f) + std::tuple{Dot(M, {tform[0],tform[1],tform[2]}),
                                   Dot(M, {tform[4],tform[5],tform[6]}),
                                   Dot(M, {tform[8],tform[9],tform[10]})} * (.1f/mlen);
        // Add the movement momentum vector (m) to the camera position (l), thereby moving the camera
        l = l + m;

        // Render graphics
        view.InitFrame();
        Render(vertices,polys, view, frustum,
            [&](auto point)
            {
                return std::apply([xyz=point-l, &tform](auto,auto,auto, auto&&... rest)
                {
                    return AsArray(-Dot(xyz, {tform[0],tform[4],tform[8]}),
                                    Dot(xyz, {tform[1],tform[5],tform[9]}),
                                    Dot(xyz, {tform[2],tform[6],tform[10]}), rest...);
                }, point);
            },
            Plot);

        auto&& pixels = view.GetPixels();
        SDL_UpdateTexture(texture, nullptr, &pixels[0], 4*W);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
