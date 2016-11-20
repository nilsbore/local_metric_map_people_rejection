#include <metaroom_detections/metaroom_detections.h>
#include <iostream>
#include <tuple>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Provide the room_xml for which you want the detections..." << endl;
    }

    string room_xml(argv[1]);

    vector<vector<tuple<float, float, float, float> > > detections;
    detections = metaroom_detections::detections_for_xml(room_xml);

    for (vector<tuple<float, float, float, float> >& image_dets : detections) {
        cout << "Detections for image:" << endl;
        for (tuple<float, float, float, float>& det : image_dets) {
            cout << "x: " << std::get<0>(det) << ", y: " << std::get<1>(det) << ", width: " << std::get<2>(det) << ", height: " << std::get<3>(det) << endl;
        }
    }

    return 0;
}
