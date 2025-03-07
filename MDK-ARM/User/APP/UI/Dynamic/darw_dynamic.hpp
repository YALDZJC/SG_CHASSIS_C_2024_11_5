#include "../UI_Queue.hpp"

class darw_Line {
public:
    static void draw_Line(int Operate, float v);
    static void draw_gyro_v(int Operate, float v);
    static void draw_move_v(int Operate, float v);
    static void draw_mcl_of(int Operate, bool is);
    static void draw_bp_of(int Operate, bool is);
};