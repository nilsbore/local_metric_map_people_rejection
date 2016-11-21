#include <boost/filesystem.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <sstream>
#include <iomanip>
#include <fstream>

using namespace std;

int main(int argc, char** argv)
{

    vector<map<string, float> > detections;
    detections.push_back(map<string, float>());
    detections.back()["x"] = 1.0f;
    detections.back()["y"] = 2.0f;
    detections.back()["width"] = 3.0f;
    detections.back()["height"] = 4.0f;
    ofstream out("test.json");
    {
        cereal::JSONOutputArchive archive_o(out);
        archive_o(detections);
    }

    return 0;
}
