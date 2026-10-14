// Lines with a trailing marker comment must be reported by check_position_types.py.
struct Body {
    float position[3];                 // BAD
    float originWorld;                 // BAD
    const float focusLocal = 0.0F;     // BAD
    std::array<float, 3> translation;  // BAD
    Eigen::Vector3f ecef;              // BAD
    float3 bodyPos;                    // BAD
};
void place(float worldPosition, double other);  // BAD
