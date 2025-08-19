#include "polygon_clip.hh"
#include "polygon_draw.hh"
#include "math.hh"
#include "view.hh"

#include <map>
#include <array>
#include <chrono>
#include <ranges>
#include <random>
#include <vector>
#include <cstdint>
#include <thread>

#include <SDL3/SDL.h>

struct Polygon
{
    // Vertex list
    std::size_t first, size; // Refers to a separate table of vertices
    unsigned    ubase, vbase, usize, vsize;
    unsigned    lubase, lvbase, lusize, lvsize;
    unsigned    flags;
    std::array<float,3> normal, tangent, bitangent;
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

        DrawPolygon<0,1,true,2,1>(points, view.Draw([&](auto&&... args) { return plot(poly, std::forward<decltype(args)>(args)...); }));
    }
}

// Regular Icosahedron
// 12 vertices, 20 faces
// See http://en.wikipedia.org/wiki/Regular_icosahedron
// And http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
template<typename CoordType>
void CreateIcosahedron(CoordType radius, auto&& AddVertex, auto&& AddFace)
{
    // Create 12 vertices of a icosahedron
    auto t          = (CoordType(1) + std::sqrt(CoordType(5))) / CoordType(2);
    auto unitradius = std::sqrt( t*t + CoordType(1) );
    auto tee = t            * radius / unitradius;
    auto one = CoordType(1) * radius / unitradius;
    // All vertexes are added first
    for(unsigned n=0; n<4; ++n) AddVertex( {(n&1?one:-one), (n&2?-tee:tee), 0} );
    for(unsigned n=0; n<4; ++n) AddVertex( {0, (n&1?one:-one), (n&2?-tee:tee)} );
    for(unsigned n=0; n<4; ++n) AddVertex( {(n&2?-tee:tee), 0, (n&1?one:-one)} );
    // Then, all faces
    for(auto f: AsArray(
        0x0B5,0x051,0x017,0x07A,0x0AB,0x159,0x5B4,0xBA2,0xA76,0x718,
        0x394,0x342,0x326,0x368,0x389,0x495,0x24B,0x62A,0x867,0x981))
        AddFace( {unsigned(f)>>8u, (f>>4u)&15u, f&15u} );
}

class TextureCanvas
{
    unsigned W,H;
    std::vector<bool> occupied;
    std::vector<std::array<float,3>> canvas;
public:
    TextureCanvas() : W{1}, H{1}, occupied(1,false), canvas(1)
    {
    }
    std::array<unsigned,2> Allocate(unsigned w,unsigned h)
    {
        for(unsigned y=0; y+h <= H; ++y)
            for(unsigned x=0; x+w <= W; )
            {
                if(unsigned occ = IsOccupied(x,y, w,h)) { x += occ-1; continue; }
                for(unsigned p=0; p<h; ++p)
                    std::fill_n(occupied.begin()+((y+p)*W+x), w, true);
                return {x,y};
            }
        // Didn't find unoccupied slot. Grow the canvas size.
        auto np = [](unsigned n) // Return next power-of-two greater than n.
        {
            unsigned leading_zeros = 0;
            for(unsigned bit=16; bit>0; bit>>=1)
                if(!(n >> bit)) leading_zeros += bit; else n >>= bit;
            return 1 << (32-leading_zeros);
        };
        Resize(std::max(W<H ? np(W) : W, w), std::max(W<H ? H : np(H), h));
        return Allocate(w,h); // tail recursion
    }
    void Resize(unsigned W2,unsigned H2)
    {
        auto oldcanvas = std::move(canvas);   canvas.resize(W2*H2);
        auto oldocc    = std::move(occupied); occupied.resize(W2*H2, false);
        for(unsigned w=std::min(W2,W), h=0; h<std::min(H2,H); ++h)
        {
            std::copy_n(oldcanvas.begin()+h*W, w, canvas.begin()+h*W2);
            std::copy_n(oldocc.begin()+h*W, w,    occupied.begin()+h*W2);
        }
        W=W2; H=H2;
    }
    unsigned IsOccupied(unsigned x,unsigned y, unsigned w,unsigned h) const
    {
        for(auto b = occupied.begin() + y*W + x, e = b + w; h > 0; --h, b+=W, e+=W)
            if(auto bad = std::find(b,e, true); bad != e)
                return std::find(bad,e, false)-b+1;
        return 0;
    }
    const auto& get(unsigned x,unsigned y) const { return canvas[y*W+x]; }
    auto&       get(unsigned x,unsigned y)       { return canvas[y*W+x]; }
} lightmap;

float hedronsizemul = 1, lightpowermul = 4;

auto CreateLevelMap(bool redo=true)
{
    // Dimensions of the map
    constexpr int size[4] { 23, 11, 13, std::max(size[0],std::max(size[1],size[2])) };
    // List of different columns (bitmasks indicating holes in vertical columns of blocks)
    static constexpr const unsigned columns[] {
        /*A*/ 0b0000000000000,
        /*B*/ 0b0000000011100,
        /*C*/ 0b0000000111100,
        /*D*/ 0b0000000111000,
        /*E*/ 0b0000000110000,
        /*F*/ 0b0000000001100,
        /*G*/ 0b0000000010000,
        /*H*/ 0b0000000000111,
        /*I*/ 0b0000000000100,
        /*J*/ 0b0000000000011,
        /*K*/ 0b0000000010011,
        /*L*/ 0b0000000001111,
        /*M*/ 0b0000001111111,
        /*N*/ 0b0000001111110,
        /*O*/ 0b0000001110111,
        /*P*/ 0b1111111111111,
        /*Q*/ 0b0000000101111,
        /*R*/ 0b0000001110000,
        /*S*/ 0b0000001111100,
        /*T*/ 0b0000001111000,
        /*U*/ 0b0000001100000,};
    // World geometry (each symbol is an index to columns[])
    static constexpr const char map[] =
        "LLLLLLPPPPQQMMMASSSTTUU"
        "LLLLLLPPPPQQMMMOMMMNNRR"
        "LLLLLLLAGALLMMMOMMMNNNN"
        "LLLLLLLJKJLLMMMOMMMNNNN"
        "HHHHHHHJKJLLMNMAMMMNNNN"
        "HHHHHHHJKJLLMNMOMMMMMMN"
        "HHHHHIHJKJLLMNMOMMMMMMN"
        "AAFFAAAAGAAAAAAAAAAAEEA"
        "AAFFAAAAGAAAAAAAAAAAEEA"
        "AABBBBBBBBBBBBBBCCDDDEA"
        "AABBBBBBBBBBBBBBCCDDDEA";
    // Function to test whether a particular cell in the world is a hole (false indicates it’s solid)
    auto hole = [&](int x,int z,int y)
        { return y>=size[2] || (x>=0 && x<size[0] && z>=0 && z<size[1] && y>=0 && ((columns[map[z*size[0]+x]-65] >> y)&1)); };
    // Slice the world along each axis in pieces that have no wall/hole transitions.
    // This is a simple way, though not optimal, to ensure that all adjacent pairs of polygons share the exact same edge,
    // something that our renderer requires to ensure gapless rendering. OpenGL has the same requirement, by the way.
    std::array<std::array<int, size[3]>, 3> slice = {};
    for(unsigned a=0; a<3; ++a)
    {
        std::array<int,3> v0{ a==0, a==1, a==2 }, v1{ a==1, a==2, a==0 }, v2{ a==2, a==0, a==1 };
        for(int Q,q,p=0; p<size[3]; p += (slice[a][p] = q))
            for(q=Q=1; p+q < size[3] && Q; q+=Q)
            for(int e=0; e < size[3] && Q; ++e)
            for(int d=0; d < size[3] && Q; ++d)
                if(std::array a = {p+q-1,d,e}, b = {p+q,d,e};
                   hole(Dot(v0,a), Dot(v1,a), Dot(v2,a)) != hole(Dot(v0,b), Dot(v1,b), Dot(v2,b)))
                    Q = 0;
    }

    std::vector<std::array<float,10>> points;
    std::vector<Polygon> poly;

    // Light sources. All of them are simply 3D points with a color.
    static const std::array<std::array<std::array<float,3>,2>,4> lights
    {{
        {{{ (15.2-1.5)*4, (7.5-1.5)*4 , ( 2.2-2.5)*4 },{ 1, 0.6, .1 }}}, // orange on the floor
        {{{ (17.3-1.5)*4, (7.5-7.5)*4 , ( 5.7-2.5)*4 },{.2,  .2,  1 }}}, // blue at the end
        {{{ ( 9.5-1.5)*4, (7.5-7  )*4 , (17.5-2.5)*4 },{96, 96, 117 }}}, // huge white in the ceiling tunnel
        {{{ ( 9.5-1.5)*4, (7.5-1.1)*4 , ( 3.9-2.5)*4 },{ 2, .4,  .2 }}}  // red in tunnel
    }};
    //static auto start = std::chrono::system_clock::now();
    //float angle = std::chrono::duration<float>(std::chrono::system_clock::now() - start).count() * 1.f;
    auto LightCoord = [/*sin = std::sin(angle), cos = std::cos(angle), center=AsArray(10*4, 3.5f*4, 0)*/](const auto& coord)
    {
        // Return the lightsource coordinate, but rotated around the approximate center of the world.
        //return center + (coord-center) * AsArray(sin+cos,cos-sin,1);
        return coord;
    };
    auto AddPoly = [&](std::initializer_list<auto> newpoints, auto&&... props)
    {
        points.insert(points.end(), newpoints);
        const auto& p = &*newpoints.begin();
        auto line2         = Normalized(AsArray(0,0,0)+p[2]-p[0]);
        auto line1         = Normalized(AsArray(0,0,0)+p[1]-p[0]);
        auto normal        = Normalized(CrossProduct(line1, line2));
        auto tangent       = Normalized(CrossProduct(normal, line1));
        auto bitangent     = Normalized(CrossProduct(normal, tangent));
        poly.emplace_back( Polygon{ points.size()-newpoints.size(),newpoints.size(),
                                    props...,
                                    normal,tangent,bitangent} );
    };
    for(auto& l: lights)
    {
        std::vector<std::array<float,3>> vert;
        CreateIcosahedron(std::cbrt(Sum(l[1]))*hedronsizemul,
            [&](std::array<float,3> v) { vert.push_back(v + LightCoord(l[0])); },
            [&](std::array<unsigned,3> p)
        {
            auto point = [&](int n) { return AsArray(vert[p[n]], n,n, 0,0, l[1]*lightpowermul); };
            AddPoly( {point(0), point(1), point(2)}, 0u,0u, 256u,256u, 0u,0u, 1u,1u, 1u );
        });
    }

    // Process each resulting cuboid.
    if(redo) for(int x=0; x<size[0]; x += slice[0][x])
    for(int z=0; z<size[1]; z += slice[1][z])
    for(int y=0; y<size[2]; y += slice[2][y])
        if(hole(x,z,y))
        {
            // Find the dimensions of this cuboid
            std::array dim { slice[0][x], slice[1][z], slice[2][y] }, pos { x,z,y };

            // And create polygons for its edges (where necessary)
            for(unsigned m=0; m<36; m+=6)
            {
                constexpr unsigned long vectors = 0b010001'001010'010100'001100'100010'100001ul;
                auto bit = [m](unsigned n) {return int(vectors >> (n+m))&1; };
                std::array<int,3> right = {bit(5),bit(4),bit(3)}, down = {bit(2),bit(1),bit(0)};

                // Test whether we need a wall on this side
                auto behind = AsArray<float>(CrossProduct(down, right));
                auto where = AsArray<float>(pos + (behind * std::max(0.f, Dot(dim,behind)-1)));
                if(std::apply(hole, where+behind)) continue;

                // Create a polygon for this side
                unsigned width = Dot(dim,right), height = Dot(dim,down);
                unsigned uvdim = 256;
                auto uv    = AsArray(0u,0u);
                auto luvdim= AsArray(16u,16u);
                auto luv   = lightmap.Allocate(width*luvdim[0], height*luvdim[1]);
                auto point = [&](unsigned w,unsigned h) {
                    return AsArray(
                        (behind*.5f + where + right*(w-.5f) + down*(h-.5f)) * 4,
                        uv+AsArray(w,h)*uvdim, luv+AsArray(w,h)*luvdim, 1,1,1);
                };
                AddPoly( { point(0,0), point(width,0), point(width,height), point(0,height) },
                           uv[0],uv[1], uvdim-1,uvdim-1, luv[0],luv[1],width*luvdim[0]-1,height*luvdim[1]-1, 0u );
            }
        }
    return std::pair(points,poly);
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
    unsigned tempbitmap[txW*txH], bitmap[txW*txH];

    std::mt19937 rnd;
    for(unsigned y=0; y<txH; ++y)
        for(unsigned x=0; x<txW; ++x)
            tempbitmap[y*txW+x] = std::clamp<int>(255*(
                  0.7 - ( (1.0-std::hypot( int(x-txW/2)/(float)(txW/2), int(y-txH/2)/(float)(txH/2) )) *0.6
                -!(x<8 || y<8 || (x+8)>=txW || (y+8)>=txH))
             * (0.1 + 0.3*std::pow((std::rand()%100)/100.0, 2.0))), 0,255);
    for(unsigned y=0; y<txH; ++y)
        for(unsigned x=0; x<txW; ++x)
            bitmap[y*txW+x] = ((tempbitmap[y*txW+x]
                               +tempbitmap[(y%txH)*txW+x]
                               +tempbitmap[(y%txH)*txW+(x+1)%txW]
                               +tempbitmap[((y+1)%txH)*txW+x]
                               +tempbitmap[((y+1)%txH)*txW+(x+1)%txW])/5*0x10101);

    auto [vertices,polys] = CreateLevelMap();

    auto decompose = [](unsigned rgb)
    {
        return AsArray<unsigned char>( rgb>>16, rgb>>8, rgb );
    };
    auto compose = [](auto rgb)
    {
        auto m = [&](auto a){return std::clamp<unsigned>(a,0,255);};
        return Dot(AsArray(m(rgb[0]),m(rgb[1]),m(rgb[2])), AsArray(65536u,256u,1u));
        //return Dot(AsArray<unsigned>(rgb), AsArray(65536u,256u,1u));
    };
    static const float bayer4x4_f[4][4] = // 4x4 ordered-dithering matrix
    {
        {  0/16.f, 8/16.f, 1/16.f, 9/16.f },
        { 12/16.f, 4/16.f,13/16.f, 5/16.f },
        {  3/16.f,11/16.f, 2/16.f,10/16.f },
        { 15/16.f, 7/16.f,14/16.f, 6/16.f }
    };
    auto Dithered = [](auto&& plot)
    {
        return [&](auto& poly,auto&& info, unsigned x,unsigned y,auto z, float u,float v, float lu,float lv, auto&&... args)
        {
            unsigned lui = lu + bayer4x4_f[y%4][x%4];
            unsigned lvi = lv + bayer4x4_f[y%4][x%4];
            unsigned ui = u + bayer4x4_f[y%4][x%4];
            unsigned vi = v + bayer4x4_f[y%4][x%4];
            return plot(poly,info, x,y,z, ui,vi, lui,lvi, args...);
        };
    };
    auto Plot = [&](auto& poly, auto,auto,auto, unsigned u,unsigned v, unsigned lu,unsigned lv, float r,float g,float b)
    {
        unsigned texel = bitmap[ (poly.vbase + (v - poly.vbase) % poly.vsize) * txW
                               + (poly.ubase + (u - poly.ubase) % poly.usize) ];
        auto rgb = std::tie(r,g,b);
        if(poly.flags == 0)
        {
            // Retrieve r,g,b from lightmap
            auto luxel = lightmap.get(std::clamp(lu, poly.lubase, poly.lubase+poly.lusize),
                                      std::clamp(lv, poly.lvbase, poly.lvbase+poly.lvsize));
            rgb = rgb * luxel;
        }
        return compose(decompose(texel) * rgb);
    };

    std::vector<std::thread> LightmapCalculator;
    for(std::size_t max=40, n=0; n<max; ++n) LightmapCalculator.emplace_back([&, n,max]
    {
        struct ViewData
        {
            View<unsigned>            view;
            std::array<unsigned,3>    vectors;   // normal, tangent, bitangent
            std::array<float,3>       viewnormal; // normal to use for lightmap weights calculation
            std::vector<Plane>        frustum;
        };
        // Create five 90° view cones.
        //constexpr std::size_t Dim=256; View<unsigned> baseview(Dim,Dim, 170.0);
        constexpr std::size_t Dim=64;
        View baseview(Dim,Dim, 90.0);
        auto basefrustum = baseview.MakeFrustum();

        ViewData lviews[] =
        {
            { baseview, AsArray<unsigned>(0,2,1), AsArray<float>(0,0,1), basefrustum },
            { baseview, AsArray<unsigned>(1,0,2), AsArray<float>(1,0,0), basefrustum },
            { baseview, AsArray<unsigned>(2,1,0), AsArray<float>(0,1,0), basefrustum },
            { baseview, AsArray<unsigned>(3,0,4), AsArray<float>(1,0,0), basefrustum },
            { baseview, AsArray<unsigned>(4,3,0), AsArray<float>(0,1,0), basefrustum }
        };
        // Normalize the weights so that the sum of weights is 1/256
        float invsum = 1.f / (256 * std::size(lviews) * Dim*Dim);
        /*
        float sum = 0;
        for(auto& viewdata: lviews)
            for(auto& w: viewdata.view.GetWeights(viewdata.viewnormal))
                sum += w;
        float invsum = 1/(256*sum);
        for(auto& viewdata: lviews)
            for(auto& w: viewdata.view.GetWeights(viewdata.viewnormal))
                w *= invsum;*/

        for(;;)
        {
            for(auto& poly: polys | std::views::filter([&](auto& v) { return v.flags==0 && std::distance(&polys[0],&v)%max==n; }))
            {
                std::array<std::array<float,3>, 6> vectors
                    { poly.normal, poly.tangent, poly.bitangent, poly.tangent*-1, poly.bitangent*-1 };

                auto displace = (poly.normal + poly.tangent + poly.bitangent) * .02f;
                DrawPolygon<5,6,false,~0u,1>(
                    std::ranges::subrange(&vertices[poly.first], &vertices[poly.first+poly.size]),
                    [&](float x,float y,float z, auto,auto, unsigned lu,unsigned lv, auto...)
                    {
                        // Camera location for viewing "up" from this surface point
                        auto l = AsArray(x,y,z) + displace;

                        std::array<float,3> color{};
                        for(auto& viewdata: lviews)
                        {
                            const auto& normal    = vectors[viewdata.vectors[0]];
                            const auto& tangent   = vectors[viewdata.vectors[1]];
                            const auto& bitangent = vectors[viewdata.vectors[2]];

                            viewdata.view.InitFrame();
                            Render(vertices,polys, viewdata.view, viewdata.frustum,
                                [&](auto point) // Transformation
                                {
                                    return std::apply([&, xyz=point-l](auto,auto,auto, auto&&... rest)
                                    {
                                        return AsArray(Dot(xyz, tangent), Dot(xyz, bitangent), Dot(xyz, normal), rest...);
                                    }, point);
                                },
                                Plot);

                            // Then collect pixels from view, sum them up
                            auto& weights = viewdata.view.GetWeights(viewdata.viewnormal);
                            auto& pixels  = viewdata.view.GetPixels();
                            #pragma omp simd
                            for(std::size_t n=0; n<Dim*Dim; ++n)
                                for(unsigned m=0; m<3; ++m)
                                    color[m] = color[m] + decompose(pixels[n])[m];// * weights[n];
                        }
                        lightmap.get(lu,lv) = color * invsum;
                    });

                // Assume dimensions (1..9), (1..9)
                //   We render to (1..8), (1..8); but we own cells (0..9), (0..9)
            }
        }
    });

    View view(W,H, 120.f/* degrees */);
    auto frustum = view.MakeFrustum();

    std::tuple r{0.f, 0.f, .2f};             // Rotation momentum vector (nonzero indicates view is still rotating)
    std::tuple m{0.f, 0.f, 0.f};             // Movement momentum vector (nonzero indicates camera is still moving)
    std::tuple l{-0.04f,0.36f,3.35f};        // Camera location (X,Y,Z) coordinate
    float aa=0.66,ab=0.63,ac=-0.28,ad=0.28;  // View rotation quaternion
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
        if(keys[SDLK_M]) if(hedronsizemul < 7) hedronsizemul *= 1.01;
        if(keys[SDLK_N]) if(hedronsizemul > 1) hedronsizemul /= 1.01;
        if(keys[SDLK_B]) if(lightpowermul < 16) lightpowermul *= 1.02;
        if(keys[SDLK_V]) if(lightpowermul >  3) lightpowermul /= 1.02;

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
                // Replaces the first three props (x,y,z) with the transformed coordinate
                // (translation, rotation). Passes the rest of the props verbatim.
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

        auto [vertices1,polys1] = CreateLevelMap(false);
        for(std::size_t n=0; n<vertices1.size(); ++n)
            vertices[n] = vertices1[n];
    }
}
