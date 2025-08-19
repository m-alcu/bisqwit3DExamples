    int rotating = 0, resizing = 0, mousex = 0, mousey = 0; bool drag = false;
    auto cursor_hand   = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_MOVE);
    auto cursor_normal = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_DEFAULT);
    auto cursor_move   = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_MOVE);
    SDL_SetCursor(cursor_normal);
