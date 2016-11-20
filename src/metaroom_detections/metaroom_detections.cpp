#include <metaroom_detections/metaroom_detections.h>
#include <boost/filesystem.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <sstream>
#include <iomanip>
#include <fstream>

namespace metaroom_detections {

using namespace std;

// each element corresponds to one image, then each element in that vector
// corresponds to one detection tuple with min_x, min_y, width, height
vector<vector<tuple<float, float, float, float> > > detections_for_xml(const string& room_xml, const string& type)
{
    boost::filesystem::path room_folder = boost::filesystem::path(room_xml).parent_path();

    vector<vector<tuple<float, float, float, float> > > rtn;

    const size_t sweep_size = 17;
    for (size_t i = 0; i < sweep_size; ++i) {
        stringstream ss;
        ss << type << setw(4) << setfill('0') << i;
        boost::filesystem::path file = room_folder / (ss.str() + ".json");

        cout << file.string() << endl;

        stringstream detections_map;
        {
            ifstream in(file.string());
            string str((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
            detections_map << "{\"value0\":  " << str << "}";
            //detections_map << "{\"value0\":  " << "[]" << "}";
            //detections_map << str;
        }
        cout << detections_map.str() << endl;
        vector<map<string, float> > detections;
        {
            cereal::JSONInputArchive archive_i(detections_map);
            archive_i(detections);
        }

        rtn.push_back(vector<tuple<float, float, float, float> >());
        for (map<string, float>& det : detections) {
            rtn.back().push_back(make_tuple(det["x"], det["y"], det["width"], det["height"]));
        }
    }

    return rtn;
}

} // namespace metaroom_detections
