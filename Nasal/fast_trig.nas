# Global namespace: fast_trig
# Pre-calculated 256-entry quarter-wave sine buffer (0 to 90 degrees)
var TABLE_SIZE = 256;
var SINE_TABLE = setsize([], TABLE_SIZE + 1);

# Populate table on module load
for (var i = 0; i <= TABLE_SIZE; i += 1) {
    var angle_rad = (i / TABLE_SIZE) * (math.pi / 2.0);
    SINE_TABLE[i] = math.sin(angle_rad);
}

# Core normalized lookup [0.0, 1.0) -> [0, 2*PI) with linear interpolation
var lookup_sin_norm = func(phase) {
    phase = phase - math.floor(phase);
    
    var scaled = phase * (TABLE_SIZE * 4);
    var idx = math.floor(scaled);
    var frac = scaled - idx;
    
    var quad = math.mod(math.floor(idx / TABLE_SIZE), 4);
    var rem  = math.mod(idx, TABLE_SIZE);
    
    var i0 = 0;
    var i1 = 0;
    var sign = 1;
    
    if (quad == 0) {
        i0 = rem;            i1 = rem + 1;            sign = 1;
    } elsif (quad == 1) {
        i0 = TABLE_SIZE - rem; i1 = TABLE_SIZE - (rem + 1); sign = 1;
    } elsif (quad == 2) {
        i0 = rem;            i1 = rem + 1;            sign = -1;
    } else {
        i0 = TABLE_SIZE - rem; i1 = TABLE_SIZE - (rem + 1); sign = -1;
    }
    
    var y0 = SINE_TABLE[i0];
    var y1 = SINE_TABLE[i1];
    return (y0 + (y1 - y0) * frac) * sign;
};

# Sine (radians)
var sin = func(rad) {
    return lookup_sin_norm(rad / (2.0 * math.pi));
};

# Cosine via +0.25 index offset (90-degree quarter-wave shift)
var cos = func(rad) {
    return lookup_sin_norm((rad / (2.0 * math.pi)) + 0.25);
};

# Tangent via quotient ratio (sin / cos) with asymptote guard
var tan = func(rad) {
    var c = cos(rad);
    if (math.abs(c) < 0.00001) {
        return (sin(rad) >= 0) ? 999999.0 : -999999.0;
    }
    return sin(rad) / c;
};
