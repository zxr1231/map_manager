#ifndef MAPMANAGER_DEPTH_EVIDENCE_H
#define MAPMANAGER_DEPTH_EVIDENCE_H

#include <cmath>
#include <cstdint>

namespace mapManager {

// A numeric depth hit is the only geometric evidence available from the
// current Gazebo OpenNI 32FC1 topic.  Its NaN does not certify a free ray.
inline bool validDepthHit(float depth, double minimum, double maximum) {
  return std::isfinite(depth) && depth >= minimum && depth < maximum;
}

inline bool validDepthHit(std::uint16_t raw, double scale, double minimum,
                          double maximum) {
  return raw != 0 && scale > 0.0 &&
         validDepthHit(static_cast<float>(raw / scale), minimum, maximum);
}

}  // namespace mapManager

#endif
