// Import the aruco module in OpenCV
#include <habitat_cv/image.h>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;
using namespace cv::aruco;
using namespace habitat_cv;


int main(){
    Images cameras = Images::read(".","png");
    // for (auto &image: cameras) {
    //     Ptr<Dictionary> dictionary = getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    //     Ptr<DetectorParameters> parameters = DetectorParameters::create();
    //     vector<vector<Point2f>> markerCorners, rejectedCandidates;
    //     json_cpp::Json_vector<int> markerIds;
    //     detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    //     cout << markerIds << endl;
    // }
    return 0;
};