#include "rasterize.hh"
#include "math.hh"
#include <array>
#include <vector>
#include <cstdint>
#include <ranges>
#include <numbers>
#include <map>

void ClipPolygon(const Plane& p, auto& points) requires std::ranges::forward_range<decltype(points)>
{
    bool keepfirst = true;

    // Process each edge of the polygon (line segment between two successive points)
    for(auto current = points.begin(); current != points.end(); )
    {
        // next = the successor of current in the ring of vertices
        auto next = std::next(current);
        if(next == points.end()) { next = points.begin(); }

        auto outside     = p.DistanceTo(*current);
        auto outsidenext = p.DistanceTo(*next);

        // If this corner is not inside the plane, keep it
        bool keep = outside >= 0;
        if(current == points.begin()) { keepfirst=keep; keep=true; }

        // If this edge of the polygon _crosses_ the plane, generate an intersection point
        if((outside < 0 && outsidenext > 0)
        || (outside > 0 && outsidenext < 0))
        {
            auto factor = outside / (outside - outsidenext);

            // Create a new point b between *current and *next like this: current + (next-current) * factor
            auto b = Mul<std::plus<void>>(Mul(Mul<std::minus<void>>(*next, *current), factor), *current);

            if(keep) { current = std::next(points.insert(std::next(current), std::move(b))); }
            else     { *current = std::move(b); ++current; }
        }
        else
        {
            // Keep or delete the original vertex
            if(keep) ++current; else current = points.erase(current);
        }
    }
    if(!keepfirst) points.erase(points.begin());
}

void TesselateAndDraw(auto&& points, auto&& GetXY, auto&&...args) requires std::ranges::random_access_range<decltype(points)>
{
    constexpr unsigned limit = 16;
    bool too_many_corners = points.size() >= limit;

    // prev[] and next[] form a double-directional linked list
    unsigned char next[limit], prev[limit];
    for(unsigned n=0; n<points.size() && n<limit; ++n)
    {
        next[n] = (n+1)==points.size() ? 0 : (n+1);
        prev[n] = n==0 ? points.size()-1 : (n-1);
    }
    for(unsigned cur = 0, remain = points.size(); remain >= 3; --remain)
    {
        unsigned p1 = next[cur], p2 = next[p1];
        unsigned a = cur, b = p1, c = p2, era = cur;
        if(remain > 3 && !too_many_corners)
        {
            unsigned m1 = prev[cur], m2 = prev[m1];
            auto [curx,cury, p1x,p1y, p2x,p2y, m1x,m1y, m2x,m2y] = std::tuple_cat(
                GetXY(points[cur]), GetXY(points[p1]), GetXY(points[p2]), GetXY(points[m1]), GetXY(points[m2]));
            // Three possible tesselations:
            //     prev2-prev1-this (score3)
            //     prev1-this-next1 (score1)
            //     this-next1-next2 (score2)
            // Score indicates how long horizontal lines there are in this triangle
            auto score1 = (std::abs(curx-p1x) + std::abs(p1x-m1x) + std::abs(m1x-curx))
                        - (std::abs(cury-p1y) + std::abs(p1y-m1y) + std::abs(m1y-cury));
            auto score2 = (std::abs(curx-p1x) + std::abs(p1x-p2x) + std::abs(p2x-curx))
                        - (std::abs(cury-p1y) + std::abs(p1y-p2y) + std::abs(p2y-cury));
            auto score3 = (std::abs(curx-m2x) + std::abs(m2x-m1x) + std::abs(m1x-curx))
                        - (std::abs(cury-m2y) + std::abs(m2y-m1y) + std::abs(m1y-cury));
            if(score1 >= score2 && score1 >= score3)      { b = p1; c = m1; /* era = cur; */ }
            else if(score2 >= score1 && score2 >= score3) { /*b = p1; c = p2;*/ era = p1; }
            else                                          { b = m2; c = m1; era = m1; }
        }
    rest:
        RasterizeTriangle(&points[a],&points[b],&points[c], GetXY, args...);
        if(too_many_corners)
        {
            b = c++;
            if(c >= remain) return;
            goto rest;
        }
        auto p = prev[era], n = next[era];
        next[p] = n;
        prev[n] = p;
        cur = n;
    }
}

class Slope
{
    float cur, step;
public:
    Slope() {}
    Slope(float begin, float end, float num_steps)
    {
        float inv_step = 1.f / num_steps;
        cur = begin;                       // Begin here
        step  = (end - begin) * inv_step;  // Stepsize = (end-begin) / num_steps
    }
    float get() const       { return cur; }
    void advance(float n=1) { cur += step*n; }
};

void DrawPolygon(auto&& points, auto&& Plot) requires std::ranges::input_range<decltype(points)>
{
    constexpr std::size_t Size = std::tuple_size_v<std::ranges::range_value_t<decltype(points)>>;

    using SlopeData = std::array<Slope, Size-1>; // All input units except y

    TesselateAndDraw(points,
        // GetXY: Retrieve std::tuple<int,int> or std::array<int,2> from a PointType
        [&](const auto& p) { return std::tuple{ int(std::get<0>(p)), int(std::get<1>(p)) }; },
        // Slope generator
        [&](const auto* from, const auto* to, int num_steps)
        {
            SlopeData result;
            // Number of steps = number of scanlines
            // Retrieve X coordinates for begin and end
            result[0] = Slope( std::get<0>(*from), std::get<0>(*to), num_steps );

            // For the Z coordinate, use the inverted value.
            float zbegin = 1.f / std::get<2>(*from), zend = 1.f / std::get<2>(*to);
            result[1] = Slope( zbegin, zend, num_steps );

            // Take the rest of the props, and use the Z coordinate (inverted) to iterate them through.
            [&]<std::size_t...p>(std::index_sequence<p...>)
            {
                // Retrieve begin and end for each prop. The +3 skips x, y, and z.
                ((result[p+2] = Slope( std::get<p+3>(*from) * zbegin, std::get<p+3>(*to) * zend, num_steps )), ...);
            }
            (std::make_index_sequence<Size-3>{});
            return result;
        },
        // Scanline function
        [&](int y, SlopeData& left, SlopeData& right)
        {
            float leftx = left[0].get(), endx = right[0].get();

            // Number of steps = number of pixels on this scanline = endx-x
            std::array<Slope, Size-2> props;
            for(unsigned p=0; p<Size-2; ++p)
            {
                props[p] = Slope( left[p+1].get(), right[p+1].get(), endx-leftx );
                props[p].advance(int(leftx)-leftx);
            }

            for(int x = leftx, e = endx; x < e; ++x) // Render each pixel
            {
                float z = 1.f / props[0].get(); // Un-invert the inverted Z coordinate
                std::apply([&](auto... args) { Plot(x,y, z, (args.get() * z) ...); }, props);

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
    const int W = 1920, H = 1080;
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
            bitmap[y*txW+x] = Sum(Mul(std::tuple{65536,256,1},{std::clamp(int((~x & ~y)&255) - d,l,255),
                                                               std::clamp(int(( x & ~y)&255) - d,l,255),
                                                               std::clamp(int((~x &  y)&255) - d,l,255)}));
        }

    // Create a box with five walls using one quad for each.
    std::vector<float> tri;
    auto addcuboid = [&](unsigned mask,
                         std::array<float,2> x, std::array<float,2> z, std::array<float,2> y,
                         std::array<float,3> c, std::array<float,3> u, std::array<float,3> v)
    {
        auto ext = [](auto m,unsigned n,unsigned b=1) { return (m >> (n*b)) & ~(~0u << b); }; // extracts bits
        // Generates: For six vertices, color(rgb), coordinate(xyz) and texture coord(uv).
        std::array p{&c[0],&c[0],&c[0], &x[0],&y[0],&z[0], &u[0],&v[0]};
        // capflag(1 bit), mask(3 bits), X(4 bits), Y(4 bits), Z(4 bits), U(4 bits), V(4 bits)
        for(unsigned m: std::array{0x960339,0xA9F339,0x436039,0x4C6F39,0x406C39,0x4F6339}) // bottom, top, four sides
            if(std::uint64_t s = (m>>23) * 0b11'000'111 * (~0llu/255); mask & m)
                for(unsigned n = 0; n < 4*8; ++n)
                    tri.push_back( p[n%8][ext(m, ext(012345444u, n%8, 3)*4 - (1+n/8)) << ext(s,n)] );
        // 12345444 = nibble indexes in "m" for each of 8 values
    };
    addcuboid(7<<20, {-10,10}, {-10,10}, {-10,10}, { 1, 1, 1}, {0,1,1},  {0,1,1});

    std::tuple r{0.f, 0.f, .2f};             // Rotation momentum vector (nonzero indicates view is still rotating)
    std::tuple m{0.f, 0.f, 0.f};             // Movement momentum vector (nonzero indicates camera is still moving)
    std::tuple l{6.9f,0.146f,-21.f};         // Camera location (X,Y,Z) coordinate
    float aa=0.094,ab=-0.11,ac=-0.021,ad=1;  // View rotation quaternion
    float tform[16]{};                       // View rotation matrix (calculated from the quaternion)
    // Generate the perspective projection and unprojection functions
    auto [PerspectiveProject, PerspectiveUnproject] = [](int W,int H, float fov)
    {
        std::array<float,2> center{ W*.5f, H*.5f }, size = center, aspect{ 1.f, W*1.f/H };
        auto scale = Mul(size, aspect, 1.f / std::tan(fov/2.f*std::numbers::pi_v<float>/180.f));
        // Converting 3D x,y,z into 2D X & Y follows this formula (rectilinear projection):
        //    X = xcenter + x * hscale / z
        //    Y = ycenter + y * vscale / z
        return std::pair{ [=](const auto& point) // PerspectiveProject function
        {
            return Mul<std::plus<void>>(Mul(scale, point, 1 / std::get<2>(point)), center);
        },
        // Doing the same in reverse, getting 3D x,y,z from 2D X & Y,
        // can be done as follows, but requires that we already know z:
        //    x = (X - xcenter) * z / hscale
        //    y = (Y - ycenter) * z / vscale
        [=](const auto& point, float z) // PerspectiveUnproject function
        {
            return std::tuple_cat(Mul<std::divides<void>>(Mul(Mul<std::minus<void>>(point, center), z), scale), std::tuple{z});
        } };
    }(W,H, 120.f /* degrees */);
    auto MakeFrustum = [&](const auto& corners) requires std::ranges::forward_range<decltype(corners)>
    {
        // znear = near clipping plane distance, zany = arbitrary z value
        constexpr float znear = 0.1, zany = 1;
        std::vector<Plane> result{ std::array<std::array<float,3>,3>{{ {0,0,znear}, {1,0,znear}, {0,1,znear}}} };
        // Iterate through all successive pairs of corner points (including last->first)
        for(auto begin = corners.begin(), end = corners.end(), current = begin; current != end; ++current)
        {
            auto next = std::next(current); if(next == end) next = begin;
            result.push_back(std::tuple{ PerspectiveUnproject(*current,zany), PerspectiveUnproject(*next,zany), std::tuple{0,0,0}} );
        }
        return result;
    };
    auto Frustum = MakeFrustum(std::initializer_list{ std::pair{0,0}, {W-1,0}, {W-1,H-1}, {0,H-1} });

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
        r = Mul<std::plus<void>>(Mul(r, .9f), Mul({0.f+(up     - down) * !alt,
                                                   0.f+(right  - left) * !alt,
                                                   0.f+(rright - rleft)}, .1f));
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
        m = Mul<std::plus<void>>(Mul(m, .9f), Mul({Dot(M, {tform[0],tform[1],tform[2]}),
                                                   Dot(M, {tform[4],tform[5],tform[6]}),
                                                   Dot(M, {tform[8],tform[9],tform[10]})}, .1f/mlen));
        // Add the movement momentum vector (m) to the camera position (l), thereby moving the camera
        l = Mul<std::plus<void>>(l, m);

        // Render graphics
        static const float bayer4x4_f[4][4] = // 4x4 ordered-dithering matrix
        {
            {  0/16.f, 8/16.f, 1/16.f, 9/16.f },
            { 12/16.f, 4/16.f,13/16.f, 5/16.f },
            {  3/16.f,11/16.f, 2/16.f,10/16.f },
            { 15/16.f, 7/16.f,14/16.f, 6/16.f }
        };
        static Uint32 pixels[W*H];
        static float  zbuffer[W*H];
        for(auto& z: pixels) z = 0;
        for(auto& z: zbuffer) z = 1e38f;
        for(unsigned p=0; p<tri.size(); p+=32)
        {

            // Collect the corner vertics of the polygon. Translate and rotate them.
            auto f = [&](unsigned p)
            {
                auto xyz = Mul<std::minus<void>>({tri[p+3],tri[p+4],tri[p+5]}, l);
                std::array r{-Dot(xyz, {tform[0],tform[4],tform[8]}),
                              Dot(xyz, {tform[1],tform[5],tform[9]}),
                              Dot(xyz, {tform[2],tform[6],tform[10]}) };
                return std::tuple{ r[0],r[1],r[2], tri[p+6]*txW,tri[p+7]*txH };
            };
            std::vector points { f(p+0), f(p+8), f(p+16), f(p+24) };
            // Optional: Discard polygons that are not facing the camera (back-face culling).
            // This is calculated by           ((p1-p0) × (p2-p0)) · p0
            // which could be optimized as...: ((p1 × p2)) · p0
            if(Dot(CrossProduct(points[1], points[2]), points[0]) > 1e-4f) continue;

            // Clip the polygon against the frustum while it’s still 3D.
            for(const Plane& p: Frustum) ClipPolygon(p, points);

            // If the polygon is no longer a surface, don’t try to render it.
            if(points.size() < 3) continue;

            // Perspective-project whatever points remain. Now it’s 2D, but with a copy of the original Z coordinate.
            for(auto& p: points)
            {
                auto [vx,vy] = PerspectiveProject(p);
                std::get<0>(p) = vx;
                std::get<1>(p) = vy;
            }

            DrawPolygon(
                std::views::all(points),
                [&](int x,int y, float z, float, float u,float v)
                {
                    // If the clipping worked properly, we no longer need range checking here.
                    if(z<zbuffer[y*W+x])
                    {
                        zbuffer[y*W+x] = z;
                        unsigned ui = u + bayer4x4_f[y%4][x%4];
                        unsigned vi = v + bayer4x4_f[y%4][x%4];
                        pixels[y*W+x] = bitmap[(ui%txW)*txW + vi%txH];
                    }
                });
        }
        SDL_UpdateTexture(texture, nullptr, pixels, 4*W);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000/60);
    }
}
