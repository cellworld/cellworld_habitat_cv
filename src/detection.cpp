#include <habitat_cv/detection.h>

namespace habitat_cv{
    Detection_list Detection_list::get_detections(const Binary_image &clean_image) {
        cv::Mat centroids;
        cv::Mat labels;
        cv::Mat stats;
        connectedComponentsWithStats(clean_image,labels,stats,centroids,4);
        Detection_list detections;
        for (int i = 0; i< stats.rows; i++)
        {
            Detection detection;
            int area = stats.at<int>(i,4);
            detection.area = area;
            detection.location.x = centroids.at<double>(i, 0);
            detection.location.y = centroids.at<double>(i, 1);
            detections.push_back(detection);
        }
        return detections;
    }
}