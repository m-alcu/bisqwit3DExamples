                case SDL_EVENT_MOUSE_MOTION:
                    mousex = ev.motion.x;
                    mousey = ev.motion.y;
                    break;
                case SDL_EVENT_MOUSE_BUTTON_DOWN: if(ev.button.button == 1) drag = true; break;
                case SDL_EVENT_MOUSE_BUTTON_UP:   if(ev.button.button == 1) drag = false; break;
                case SDL_EVENT_KEY_DOWN:
                case SDL_EVENT_KEY_UP:
                    if(ev.key.key == SDLK_ESCAPE) interrupted = true;
                    int rotate = (ev.key.key == SDLK_A) ? 1 : (ev.key.key == SDLK_D) ? 2 : 0;
                    int resize = (ev.key.key == SDLK_Q) ? 1 : (ev.key.key == SDLK_E) ? 2 : 0;
                    resize |=    (ev.key.key == SDLK_Z) ? 4 : (ev.key.key == SDLK_C) ? 8 : 0;
                    if(ev.type == SDL_EVENT_KEY_DOWN) rotating |= rotate; else rotating &= ~rotate;
                    if(ev.type == SDL_EVENT_KEY_DOWN) resizing |= resize; else resizing &= ~resize;
        }
        if(rotating)
        {
            float angle = ((rotating&2) - 2*(rotating&1)) * 0.01;
            float s     = std::sin(angle), c = std::cos(angle);
            for(auto& t: triangles)
                for(auto& p: t)
                {
                    float x = (p[0] - W/2)*c - (p[1] - H/2)*s;
                    float y = (p[0] - W/2)*s + (p[1] - H/2)*c;
                    p[0] = x + W/2;
                    p[1] = y + H/2;
                }
        }
        if(resizing)
        {
            float yfactor = std::pow(1.04f, ((resizing&2) - 2*(resizing&1))/2.f);
            float xfactor = std::pow(1.04f, ((resizing&8) - 2*(resizing&4))/8.f);
            for(auto& t: triangles)
                for(auto& p: t)
                {
                    p[0] = (p[0] - W/2) * xfactor + W/2;
                    p[1] = (p[1] - H/2) * yfactor + H/2;
                }
        }
        // Works perfectly as long as it stays as a parallelogram!

        // Check what's under mouse cursor
        if(1)
        {
            int width, height;
            SDL_GetWindowSize(window, &width, &height);
            int mx = mousex * W / width, my = mousey * H / height;

            bool draggable = false;
            for(auto& t: triangles)
                for(auto& p: t)
                {
                    auto& x = p[0], &y = p[1];
                    if((x-mx)*(x-mx) + (y-my)*(y-my) < 26*26)
                    {
                        draggable = true;
                        if(drag) { x = mx; y = my; }
                    }
                }
            SDL_SetCursor(draggable ? (drag ? cursor_move : cursor_hand) : cursor_normal);
