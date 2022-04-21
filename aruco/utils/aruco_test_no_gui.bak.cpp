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
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#if CV_MAJOR_VERSION >= 4
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif
using namespace std;
using namespace cv;
using namespace aruco;

MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage, TheInputImageGrey, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos, void *);
string dictionaryString;
int iDetectMode = 0, iMinMarkerSize = 0, iCorrectionRate = 0, iShowAllCandidates = 0, iEnclosed = 0, iThreshold, iCornerMode, iDictionaryIndex, iTrack = 0;

int waitTime = 0;
bool showMennu = false, bPrintHelp = false, isVideo = false;
class CmdLineParser {
    int argc;
    char **argv;

public:
    CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}
    bool operator[](string param) {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param) idx = i;
        return (idx != -1);
    }
    string operator()(string param, string defvalue = "-1") {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param) idx = i;
        if (idx == -1) return defvalue;
        else
            return (argv[idx + 1]);
    }
};

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
};

TimerAvrg Fps;

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
    cout << "Imagesize=" << imres.size() << endl;
    return imres;
}

/************************************
 *
 *
 *
 *
 ************************************/
void setParamsFromGlobalVariables(aruco::MarkerDetector &md) {


    md.setDetectionMode((DetectionMode) iDetectMode, float(iMinMarkerSize) / 1000.);
    md.getParameters().setCornerRefinementMethod((aruco::CornerRefinementMethod) iCornerMode);

    md.getParameters().detectEnclosedMarkers(iEnclosed);
    md.getParameters().ThresHold = iThreshold;
    md.getParameters().trackingMinDetections = (iTrack ? 3 : 0);
    if (aruco::Dictionary::getTypeFromString(md.getParameters().dictionary) != Dictionary::CUSTOM)
        md.setDictionary((aruco::Dictionary::DICT_TYPES) iDictionaryIndex, float(iCorrectionRate) / 10.);// sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
}

/************************************
 *
 *
 *
 *
 ************************************/
cv::Size parseSize(const string &strsize) {
    if (strsize.size() == 0) return cv::Size(-1, -1);
    cv::Size s;
    string ssaux = strsize;
    for (auto &c : ssaux) {
        if (c == ':') {
            c = ' ';
        }
    }
    stringstream sstr;
    sstr << ssaux;
    if (sstr >> s.width >> s.height)
        return s;
    return cv::Size(-1, -1);
}

int main(int argc, char **argv) {
    try {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"]) {
            cerr << "Invalid number of arguments" << endl;
            cerr << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)]|net[-http://admin:admin@192.168.3.76:8081/video]) [-c camera_params.yml] [-s  marker_size_in_meters] [-d "
                    "dictionary:ALL_DICTS by default] [-h] [-ws w:h] [-skip frames]"
                 << endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;
            return false;
        }

        ///////////  PARSE ARGUMENTS
        string TheInputVideo = argv[1];
        string paramfile = argv[2];
        // read camera parameters if passed
        if (cml["-c"])
            TheCameraParameters.readFromXMLFile(cml("-c"));

        float TheMarkerSize = std::stof(cml("-s", "-1"));
        //resize factor
        float resizeFactor = stof(cml("-rf", "1"));

        iMinMarkerSize = stof(cml("-mms", "0.0"));


        ///////////  OPEN VIDEO
        // read from camera or from  file
        if (TheInputVideo.find("live") != string::npos) {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo.find(":") != string::npos) {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer.open(vIdx);
            waitTime = 10;
            isVideo = true;
        } else if (TheInputVideo.find("net") != string::npos) {
            // check if the :idx is here
            char url[100];
            char cad[100];
            if (TheInputVideo.find("-") != string::npos) {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), '-', ' ');
                sscanf(TheInputVideo.c_str(), "%s %s", cad, url);
            }
            cout << "Opening camera url " << url << endl;
            TheVideoCapturer.open(url);
            waitTime = 10;
            isVideo = true;
        } else {
            TheVideoCapturer.open(TheInputVideo);
            if (TheVideoCapturer.get(CV_CAP_PROP_FRAME_COUNT) >= 2) isVideo = true;
            if (cml["-skip"])
                TheVideoCapturer.set(CV_CAP_PROP_POS_FRAMES, stoi(cml("-skip")));
        }
        // check video is open
        if (!TheVideoCapturer.isOpened())
            throw std::runtime_error("Could not open video");

        ///// CONFIGURE DATA
        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;
        if (TheCameraParameters.isValid())
            TheCameraParameters.resize(TheInputImage.size());
        dictionaryString = cml("-d", "ALL_DICTS");
        iDictionaryIndex = (uint64_t) aruco::Dictionary::getTypeFromString(dictionaryString);
        MDetector.setDictionary(dictionaryString, float(iCorrectionRate) / 10.);// sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
        iThreshold = MDetector.getParameters().ThresHold;
        iCornerMode = MDetector.getParameters().cornerRefinementM;


        setParamsFromGlobalVariables(MDetector);


        // go!

        do {

            TheVideoCapturer.retrieve(TheInputImage);
            std::cout << "Frame:" << TheVideoCapturer.get(CV_CAP_PROP_POS_FRAMES) << std::endl;
            TheInputImage = resizeImage(TheInputImage, resizeFactor);
            // copy image
            Fps.start();
            TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
            Fps.stop();
            // chekc the speed by calculating the mean speed of all iterations
            cout << "\rTime detection=" << Fps.getAvrg() * 1000 << " milliseconds nmarkers=" << TheMarkers.size() << " images resolution=" << TheInputImage.size() << std::endl;

            for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                // cout << TheMarkers[i] << endl;
                double position[3];
                double orientation[4];
                TheMarkers[i].OgreGetPoseParameters(position, orientation);

                cout << "id = " << TheMarkers[i].id << ": ";
                cout << "pos = (" << position[0] << ", " << position[1] << ", " << position[2] << "); ";
                cout << "orient = (" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << "); " << endl;
                TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2, true);
            }

            // draw a 3d cube in each marker if there is 3d info
            if (TheCameraParameters.isValid() && TheMarkerSize > 0)
                for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                    CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                }

            if (isVideo)
                if (TheVideoCapturer.grab() == false) break;
        } while (true);
    } catch (std::exception &ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
