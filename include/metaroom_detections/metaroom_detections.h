#ifndef METAROOM_DETECTIONS_H
#define METAROOM_DETECTIONS_H

#include <vector>
#include <string>

namespace metaroom_detections {

// each element corresponds to one image, then each element in that vector
// corresponds to one detection tuple with min_x, min_y, width, height
std::vector<std::vector<std::tuple<float, float, float, float> > > detections_for_xml(const std::string& room_xml,
                                                                                      const std::string& type = "intermediate_detection");

} // namespace metaroom_detections

#endif // METAROOM_DETECTIONS_H
