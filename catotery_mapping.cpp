std::map<Color, std_msgs::ColorRGBA> getColorMap()
{
    static const std::map<Color, std_msgs::ColorRGBA> color_map = {
        {RED, {1.0, 0.0, 0.0, 1.0}},
        {GREEN, {0.0, 1.0, 0.0, 1.0}},
        {BLUE, {0.0, 0.0, 1.0, 1.0}},
        {YELLOW, {1.0, 1.0, 0.0, 1.0}},
        {MAGENTA, {1.0, 0.0, 1.0, 1.0}},
        {CYAN, {0.0, 1.0, 1.0, 1.0}},
        {ORANGE, {1.0, 0.5, 0.0, 1.0}},
        {PURPLE, {0.5, 0.0, 0.5, 1.0}},
        {PINK, {1.0, 0.75, 0.8, 1.0}},
        {LIME, {0.75, 1.0, 0.0, 1.0}},
        {TEAL, {0.0, 0.5, 0.5, 1.0}},
        {LAVENDER, {0.9, 0.9, 0.98, 1.0}},
        {BROWN, {0.6, 0.4, 0.2, 1.0}},
        {GRAY, {0.5, 0.5, 0.5, 1.0}},
        {WHITE, {1.0, 1.0, 1.0, 1.0}}
    };

    return color_map;
}
