/**
Copyright 2020 Rafael Muñoz Salinas. All rights reserved.

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "aruco.h"
#include "cvdrawingutils.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if CV_MAJOR_VERSION >= 4
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif
using namespace cv;
using namespace aruco;

struct TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;
    TimerAvrg(int _n = 30) {
        n = _n;
        times.reserve(n);
    }
    inline void start() { begin = std::chrono::high_resolution_clock::now(); }
    inline void stop() {
        end = std::chrono::high_resolution_clock::now();
        double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
        if (times.size() < n) times.push_back(duration);
        else {
            times[curr] = duration;
            curr++;
            if (curr >= times.size()) curr = 0;
        }
    }
    double getAvrg() {
        double sum = 0;
        for (auto t : times) sum += t;
        return sum / double(times.size());
    }
} Fps;

cv::Mat resize(const cv::Mat &in, cv::Size s) {
    if (s.width == -1 || s.height == -1) return in;
    cv::Mat im2;
    cv::resize(in, im2, s);
    return im2;
}

cv::Mat resize(const cv::Mat &in, int width) {
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}

cv::Mat resizeImage(cv::Mat &in, float resizeFactor) {
    if (fabs(1 - resizeFactor) < 1e-3) return in;
    float nc = float(in.cols) * resizeFactor;
    float nr = float(in.rows) * resizeFactor;
    cv::Mat imres;
    cv::resize(in, imres, cv::Size(nc, nr));
    std::cout << "Imagesize=" << imres.size() << std::endl;
    return imres;
}

int main() {

    // float resizeFactor = stof(cml("-rf", "1"));
    float resizeFactor = 1;
    // float TheMarkerSize = std::stof(cml("-s", "-1"));
    float TheMarkerSize = -1;

    try {
        MarkerDetector MDetector;
        CameraParameters TheCameraParameters;

        Mat TheInputImage = imread("~/qr.png", IMREAD_UNCHANGED), TheInputImageCopy;
        TheInputImage = resizeImage(TheInputImage, resizeFactor);

        // go!
        Fps.start();
        std::vector<Marker> TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
        Fps.stop();

        // chekc the speed by calculating the mean speed of all iterations
        std::cout << "\rTime detection=" << Fps.getAvrg() * 1000 << " milliseconds nmarkers=" << TheMarkers.size() << " images resolution=" << TheInputImage.size() << std::endl;

        for (unsigned int i = 0; i < TheMarkers.size(); i++) {
            // cout << TheMarkers[i] << endl;
            double position[3];
            double orientation[4];
            TheMarkers[i].OgreGetPoseParameters(position, orientation);
            std::cout << "id = " << TheMarkers[i].id << ": " << std::endl
                      << "pos = (" << position[0] << ", " << position[1] << ", " << position[2] << "); " << std::endl
                      << "orient = (" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << "); " << std::endl;
            TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2, true);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (TheCameraParameters.isValid() && TheMarkerSize > 0)
            for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
            }
    } catch (std::exception &ex) {
        std::cout << "Exception :" << ex.what() << std::endl;
    }
}
