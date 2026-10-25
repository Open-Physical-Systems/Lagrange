// Nothing here may be reported by check_position_types.py.
struct Body {
    double position[3];
    std::array<double, 3> originWorld;
    float positionRender[3];  // render frame, computed last on the CPU
    float mass;
    float temperatureLocalK;  // not a position: "local" is not the last word
    float origin_scale;       // lagrange-lint: allow-float-position (fixture)
    // float commentedPosition;
    float composite;  // "position" only as part of another word
};
