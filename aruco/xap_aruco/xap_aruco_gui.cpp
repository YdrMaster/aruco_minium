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
#include <string>
#include <stdexcept>
#include <xap_external_protocol.h>
#include <xap_external_protocol_decode.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>

#if  CV_MAJOR_VERSION >= 4
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif
using namespace std;
using namespace cv;
using namespace aruco;

MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageGrey, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos, void*);
string dictionaryString;
int iDetectMode=0,iMinMarkerSize=0,iCorrectionRate=0,iShowAllCandidates=0,iEnclosed=0,iThreshold,iCornerMode,iDictionaryIndex,iTrack=0;

int waitTime = 0;
bool showMennu=false,bPrintHelp=false,isVideo=false;
class CmdLineParser {
    int argc;
    char** argv;
public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv) {}   bool operator[](string param)    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;
        return (idx != -1);
    }    string operator()(string param, string defvalue = "-1")    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;
        if (idx == -1)return defvalue;
        else return (argv[idx + 1]);
    }
};
struct   TimerAvrg {
    std::vector<double> times;
    size_t curr=0,n;
    std::chrono::high_resolution_clock::time_point begin,end;
    TimerAvrg(int _n=30) {
        n=_n;
        times.reserve(n);
    } inline void start() {
        begin= std::chrono::high_resolution_clock::now();
    } inline void stop() {
        end= std::chrono::high_resolution_clock::now();
        double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
        if ( times.size()<n) times.push_back(duration);
        else {
            times[curr]=duration;
            curr++;
            if (curr>=times.size()) curr=0;
        }
    } double getAvrg() {
        double sum=0;
        for(auto t:times) sum+=t;
        return sum/double(times.size());
    }
};

TimerAvrg Fps;

cv::Mat resize(const cv::Mat& in, cv::Size s) {
    if(s.width==-1 || s.height==-1)return in;
    cv::Mat im2;
    cv::resize(in, im2, s);
    return im2;
}



cv::Mat resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}
cv::Mat resizeImage(cv::Mat &in,float resizeFactor) {
    if (fabs(1-resizeFactor)<1e-3 )return in;
    float nc=float(in.cols)*resizeFactor;
    float nr=float(in.rows)*resizeFactor;
    cv::Mat imres;
    cv::resize(in,imres,cv::Size(nc,nr));
    cout<<"Imagesize="<<imres.size()<<endl;
    return imres;
}
/************************************
 *
 *
 *
 *
 ************************************/
void setParamsFromGlobalVariables(aruco::MarkerDetector &md) {


    md.setDetectionMode((DetectionMode)iDetectMode,float(iMinMarkerSize)/1000.);
    md.getParameters().setCornerRefinementMethod( (aruco::CornerRefinementMethod) iCornerMode);

    md.getParameters().detectEnclosedMarkers(iEnclosed);
    md.getParameters().ThresHold=iThreshold;
    md.getParameters().trackingMinDetections=(iTrack?3:0);
    if ( aruco::Dictionary::getTypeFromString( md.getParameters().dictionary)!=Dictionary::CUSTOM)
        md.setDictionary((aruco::Dictionary::DICT_TYPES) iDictionaryIndex,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
}

void createMenu() {
    cv::createTrackbar("Dictionary", "menu", &iDictionaryIndex, 13, cvTackBarEvents);
    cv::createTrackbar("DetectMode", "menu", &iDetectMode, 2, cvTackBarEvents);
    cv::createTrackbar("CornerMode", "menu", &iCornerMode, 2, cvTackBarEvents);
    cv::createTrackbar("Track", "menu", &iTrack,1, cvTackBarEvents);

    cv::createTrackbar("MinMarkerSize", "menu", &iMinMarkerSize, 1000, cvTackBarEvents);
    cv::createTrackbar("Threshold", "menu", &iThreshold, 40, cvTackBarEvents);
    cv::createTrackbar("ErrorRate", "menu", &iCorrectionRate, 10, cvTackBarEvents);
    cv::createTrackbar("Enclosed", "menu", &iEnclosed, 1, cvTackBarEvents);
    cv::createTrackbar("ShowAll", "menu", &iShowAllCandidates, 1, cvTackBarEvents);
    iThreshold=MDetector.getParameters().ThresHold;
    iCornerMode= MDetector.getParameters().cornerRefinementM;
}

void putText(cv::Mat &im,string text,cv::Point p,float size) {
    float fact=float(im.cols)/float(640);
    if (fact<1) fact=1;

    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(125,255,255),1*fact);

}
void printHelp(cv::Mat &im)
{
    float fs=float(im.cols)/float(1200);

    putText(im,"'m': show/hide menu",cv::Point(10,fs*60),fs*0.5f);
    putText(im,"'s': start/stop video capture",cv::Point(10,fs*80),fs*0.5f);
    putText(im,"'w': write image to file",cv::Point(10,fs*100),fs*0.5f);
    putText(im,"'t': do a speed test",cv::Point(10,fs*120),fs*0.5f);
    putText(im,"'f': saves current configuration to file 'arucoConfig.yml'",cv::Point(10,fs*140),fs*0.5f);
}

void printInfo(cv::Mat &im) {
    float fs=float(im.cols)/float(1200);
    putText(im,"fps="+to_string(1./Fps.getAvrg()),cv::Point(10,fs*20),fs*0.5f);
    putText(im,"'h': show/hide help",cv::Point(10,fs*40),fs*0.5f);
    if(bPrintHelp) printHelp(im);
}

void printMenuInfo() {
    cv::Mat image(200,400,CV_8UC3);
    image=cv::Scalar::all(255);
    string str="Dictionary="+aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex) ;

    cv::putText(image,str,cv::Size(10,20),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);

    str="Detection Mode="+MarkerDetector::Params::toString(MDetector.getParameters().detectMode);
    cv::putText(image,str,cv::Size(10,40),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
    str="Corner Mode="+MarkerDetector::Params::toString(MDetector.getParameters().cornerRefinementM);;
    cv::putText(image,str,cv::Size(10,60),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
    cv::imshow("menu",image);
}


/************************************
 *
 *
 *
 *
 ************************************/
cv::Size parseSize(const string &strsize  ) {
    if(strsize.size()==0)return cv::Size(-1,-1);
    cv::Size s;
    string ssaux=strsize;
    for(auto &c:ssaux) {
        if(c==':') {
            c=' ';
        }
    }
    stringstream sstr;
    sstr<<ssaux;
    if( sstr>>s.width>>s.height)
        return s;
    return cv::Size(-1,-1);

}

bool sendMessage(int serial_fd, const uint8_t msgClass, const uint8_t msgID, const uint8_t *payload, const uint16_t length)
{
    xap_external_header_t header = {XAP_EXTERNAL_SYNC1, XAP_EXTERNAL_SYNC2, msgID, msgClass, length};
    xap_external_crc_end_t crcEnd = {0, XAP_EXTERNAL_END};

    // Calculate crc
    xap_external_crc_init(&crcEnd.crc);
    crcEnd.crc = xap_external_crc_table_accumulate_buffer((uint8_t *)&header, 2, sizeof(header) - 2, crcEnd.crc);// skip 2 sync bytes

    if (payload != NULL) {
        crcEnd.crc = xap_external_crc_table_accumulate_buffer(payload, 0, length, crcEnd.crc);
    }

    // Send message
    if (sizeof(header) != write(serial_fd, (void *)&header, sizeof(header))) {
        return false;
    }

    if (payload && (length != write(serial_fd, payload, length))) {
        return false;
    }

    if (sizeof(crcEnd) != write(serial_fd, (void *)&crcEnd, sizeof(crcEnd))) {
        return false;
    }

    return true;
}

typedef struct {
    uint32_t org_timestamp;
    uint32_t rec_timestamp;
    uint32_t xmt_timestamp;

    uint32_t event_count;

    float theta_least;
    float theta_most;
    float theta_mean;
    float theta_M2;
    float theta_rms;

    float delta_least;
    float delta_most;
    float delta_mean;
    float delta_M2;
    float delta_rms;

} xapTimeSyncState_t;

int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"])
        {
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

        int serial_fd = -1;
        if (cml["-q"])
        {
            const char* serial_name = cml("-q").data();
            serial_fd = open(serial_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

            if (serial_fd < 0) {
                cout << "open failed （" << errno << ")" << endl;
                return -1;
            }

            struct termios uart_config;

            int termios_state;

            /* fill the struct for the new configuration */
            tcgetattr(serial_fd, &uart_config);

            /* clear ONLCR flag (which appends a CR for every LF) */
            uart_config.c_oflag &= ~ONLCR;

            /* no parity, one stop bit */
            uart_config.c_cflag &= ~(CSTOPB | PARENB);

            unsigned speed = B921600;

            /* set baud rate */
            if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
                cout << "CFG: " << termios_state << " ISPD" << endl;
                return -1;
            }

            if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
                cout << "CFG: " << termios_state << " OSPD" << endl;
                return -1;
            }

            if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
                cout << "baud " << termios_state << " ATTR" << endl;
                return -1;
            }
        }

        /* 串口通讯 */
        xap_external_protocol_decoder_t xap_external_protocol_decoder;  // 协议解析器
        uint8_t buff_in[512];       // 串口接收数据缓冲区
        uint8_t buff_out[512];      // 串口发送数据缓冲区

        float TheMarkerSize = std::stof(cml("-s", "-1"));
        //resize factor
        float resizeFactor=stof(cml("-rf","1"));

        iMinMarkerSize=stof(cml("-mms","0.0"));


        ///////////  OPEN VIDEO
        // read from camera or from  file
        if (TheInputVideo.find("liveusb") != string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo.find(":") != string::npos)
            {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer.open(vIdx);
            waitTime = 10;
            isVideo=true;
        }
        else if(TheInputVideo.find("net") != string::npos) {
            char url[100];
            char cad[100];
            if (TheInputVideo.find("-") != string::npos)
            {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), '-', ' ');
                sscanf(TheInputVideo.c_str(), "%s %s", cad, url);
            }
            cout << "Opening camera url " << url << endl;
            TheVideoCapturer.open(url);
            waitTime = 10;
            isVideo=true;
        }
        else {
            TheVideoCapturer.open(TheInputVideo);
            if ( TheVideoCapturer.get(CV_CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
            if(cml["-skip"])
                TheVideoCapturer.set(CV_CAP_PROP_POS_FRAMES,stoi(cml("-skip")));

        }
        // check video is open
        if (!TheVideoCapturer.isOpened())
            throw std::runtime_error("Could not open video");


        //create windows
        if(cml["-fs"]) {
            cv::namedWindow("in", cv::WINDOW_FULLSCREEN);
            cv::namedWindow("thres", cv::WINDOW_FULLSCREEN);
        }
        else if(cml["-ws"])
        {
            cv::namedWindow("in",cv::WINDOW_NORMAL);
            cv::Size s=parseSize(cml("-ws"));
            cv::resizeWindow("in",s.width,s.height);
            cv::namedWindow("thres",cv::WINDOW_NORMAL);
            resizeWindow("thres",s.width,s.height);

        }

        else {
            cv::namedWindow("in",cv::WINDOW_NORMAL);
            cv::resizeWindow("in",640,480);
            float w=std::min(int(1920),int(TheInputImage.cols));
            float f=w/float(TheInputImage.cols);
            resizeWindow("in",w,float(TheInputImage.rows)*f);
        }

        ///// CONFIGURE DATA
        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;
        if (TheCameraParameters.isValid())
            TheCameraParameters.resize(TheInputImage.size());
        dictionaryString=cml("-d", "ALL_DICTS");
        iDictionaryIndex=(uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
        MDetector.setDictionary(dictionaryString,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
        iThreshold=MDetector.getParameters().ThresHold;
        iCornerMode= MDetector.getParameters().cornerRefinementM;


        setParamsFromGlobalVariables(MDetector);


        // go!
        char key = 0;
        int index = 0,indexSave=0;
        struct timespec sync_timestamp = {0,0};

        xapTimeSyncState_t xapTimeSyncState = {0};

        /* 串口协议解析器初始化 */
        xapExternalProtocolDecodeInit(&xap_external_protocol_decoder);
        // capture until press ESC or until the end of the video

        do
        {

            TheVideoCapturer.retrieve(TheInputImage);
            std::cout<<"Frame:"<<TheVideoCapturer.get(CV_CAP_PROP_POS_FRAMES)<<std::endl;
            TheInputImage=resizeImage(TheInputImage,resizeFactor);
            // copy image
            Fps.start();
            TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
            Fps.stop();
            // chekc the speed by calculating the mean speed of all iterations
            cout << "\rTime detection=" << Fps.getAvrg()*1000 << " milliseconds nmarkers=" << TheMarkers.size() <<" images resolution="<<TheInputImage.size() <<std::endl;

            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);

            if (iShowAllCandidates) {
                auto candidates=MDetector.getCandidates();
                for(auto cand:candidates)
                    Marker(cand,-1).draw(TheInputImageCopy, Scalar(255, 0, 255));
            }

            for (unsigned int i = 0; i < TheMarkers.size(); i++)
            {
                if(TheMarkers[i].id == 7) {  // cout << TheMarkers[i] << endl;
                    double position[3];
                    double orientation[4];
                    TheMarkers[i].OgreGetPoseParameters(position, orientation);

                    cout << "id = " << TheMarkers[i].id << ": " ;
                    cout << "pos = (" << position[0] << ", " << position[1] << ", " << position[2] << "); ";
                    cout << "orient = (" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << "); " << endl;
                    TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255),2,true);

                    if(serial_fd > 0) {
                        xap_external_mcp_nav_aruco_t xap_external_mcp_nav_aruco;

                        xap_external_mcp_nav_aruco.markerID = TheMarkers[i].id;
                        xap_external_mcp_nav_aruco.x = position[0];
                        xap_external_mcp_nav_aruco.y = position[1];
                        xap_external_mcp_nav_aruco.z = position[2];

                        xap_external_mcp_nav_aruco.q0 = orientation[0];
                        xap_external_mcp_nav_aruco.q1 = orientation[1];
                        xap_external_mcp_nav_aruco.q2 = orientation[2];
                        xap_external_mcp_nav_aruco.q3 = orientation[3];

                        sendMessage(serial_fd, XAP_EXTERNAL_CLASS_MCP_NAV, XAP_EXTERNAL_MCP_NAV_ARUCO, (const uint8_t*)&xap_external_mcp_nav_aruco, sizeof(xap_external_mcp_nav_aruco));
                    }
                }
            }

            int dataNum = read(serial_fd, buff_in, sizeof(buff_in));
            int cmd_id = -1;
            for (int i = 0; i < dataNum; ++i)
            {
                cmd_id = xapExternalProtocolParseChar(&xap_external_protocol_decoder, buff_in[i]);
                switch (cmd_id >> 8)
                {
                case XAP_EXTERNAL_CLASS_COMMON_TIME:

                    switch (cmd_id & 0x00FF)
                    {
                    case XAP_EXTERNAL_COMMON_TIME_SYNC:
                        clock_gettime(CLOCK_MONOTONIC, &sync_timestamp);

                        xapTimeSyncState.rec_timestamp = sync_timestamp.tv_sec * 1000 + sync_timestamp.tv_sec / 1e6;
                        xapTimeSyncState.org_timestamp =
                            xap_external_protocol_decoder.rx_buffer.xap_external_common_time_sync.xmt_timestamp;

                        /* 设置时间同步帧消息 */
                        xap_external_common_time_sync_t xap_external_common_time_sync;

                        xap_external_common_time_sync.org_timestamp = xapTimeSyncState.org_timestamp;
                        xap_external_common_time_sync.rec_timestamp = xapTimeSyncState.rec_timestamp;
                        xap_external_common_time_sync.xmt_timestamp = sync_timestamp.tv_sec * 1000 + sync_timestamp.tv_sec / 1e6;

                        /* 记录时间同步帧发送时刻 */
                        xapTimeSyncState.xmt_timestamp = xap_external_common_time_sync.xmt_timestamp;

                        sendMessage(serial_fd, XAP_EXTERNAL_CLASS_COMMON_TIME, XAP_EXTERNAL_COMMON_TIME_SYNC, (const uint8_t*)&xap_external_common_time_sync, sizeof(xap_external_common_time_sync));

                        if (xapTimeSyncState.xmt_timestamp ==
                                xap_external_protocol_decoder.rx_buffer.xap_external_common_time_sync.org_timestamp) {
                            if (xap_external_protocol_decoder.rx_buffer.xap_external_common_time_sync.rec_timestamp != 0
                                    & xap_external_protocol_decoder.rx_buffer.xap_external_common_time_sync.xmt_timestamp != 0) {
                            }

                            int32_t T1 = xapTimeSyncState.xmt_timestamp;
                            int32_t T2 = xap_external_protocol_decoder.rx_buffer.xap_external_common_time_sync.rec_timestamp;
                            int32_t T3 = xapTimeSyncState.org_timestamp;
                            int32_t T4 = xapTimeSyncState.rec_timestamp;

                            float theta = ((T2 - T1) + (T3 - T4)) / 2 / 1e3f;
                            float delta = ((T4 - T1) - (T3 - T2)) / 2 / 1e3f;

                            xapTimeSyncState.event_count ++;

                            float d_theta = theta - xapTimeSyncState.theta_mean;
                            xapTimeSyncState.theta_mean += d_theta / xapTimeSyncState.event_count;
                            xapTimeSyncState.theta_M2 += d_theta * (theta - xapTimeSyncState.theta_mean);
                            xapTimeSyncState.theta_least = min(d_theta, xapTimeSyncState.theta_least);
                            xapTimeSyncState.theta_most = max(d_theta, xapTimeSyncState.theta_most);
                            xapTimeSyncState.theta_rms = sqrtf(xapTimeSyncState.theta_M2 / (xapTimeSyncState.event_count -
                                                               1));

                            float d_delta = delta - xapTimeSyncState.delta_mean;
                            xapTimeSyncState.delta_mean += d_delta / xapTimeSyncState.event_count;
                            xapTimeSyncState.delta_M2 += d_delta * (delta - xapTimeSyncState.delta_mean);
                            xapTimeSyncState.delta_least = min(d_delta, xapTimeSyncState.delta_least);
                            xapTimeSyncState.delta_most = max(d_delta, xapTimeSyncState.delta_most);
                            xapTimeSyncState.delta_rms = sqrtf(xapTimeSyncState.delta_M2 / (xapTimeSyncState.event_count -
                                                               1));
                        }
                        break;

                    default:
                        break;
                    }
                    break;

                default:
                    break;
                }
            }

            // draw a 3d cube in each marker if there is 3d info
            if (TheCameraParameters.isValid() && TheMarkerSize > 0)
                for (unsigned int i = 0; i < TheMarkers.size(); i++)
                {
                    if(TheMarkers[i].id == 7) {
                        CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                        CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                    }
                }

            // DONE! Easy, right?
            // show input with augmented information and  the thresholded image
            printInfo(TheInputImageCopy);
            if(showMennu)printMenuInfo();

            cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));

            cv::imshow("in",  TheInputImageCopy);

            key = cv::waitKey(waitTime);  // wait for key to be pressed
            if (key == 's')
                waitTime = waitTime == 0 ? 10 : 0;
            if (key == 'w') { //writes current input image
                string number=std::to_string(indexSave++);
                while(number.size()!=3)number="0"+number;
                string imname="arucoimage"+number+".png";
                cv::imwrite(imname,TheInputImageCopy);
                cout<<"saved "<<imname<<endl;
                imname="orgimage"+number+".png";
                cv::imwrite(imname,TheInputImage);
                cout<<"saved "<<imname<<endl;
                imname="thresimage"+number+".png";
                cv::imwrite(imname,MDetector.getThresholdedImage());

            }
            if (key=='m') {
                if (showMennu)                     cv::destroyWindow("menu");
                else {
                    cv::namedWindow("menu",cv::WINDOW_NORMAL);
                    cv::resizeWindow("menu",640,480);
                    createMenu();
                    printMenuInfo();
                }
                showMennu=!showMennu;
            }
            if (key=='h')bPrintHelp=!bPrintHelp;

            if (key=='t') { //run a deeper speed test

                for(int t=0; t<30; t++) {
                    // Detection of markers in the image passed
                    Fps.start();
                    TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
                    Fps.stop();
                    // chekc the speed by calculating the mean speed of all iterations
                }
                printInfo(TheInputImageCopy);
            }
            if(key=='f') {
                cerr<<"Configuration saved to arucoConfig.yml"<<endl;
                MDetector.saveParamsToFile("arucoConfig.yml");
            }
            index++;  // number of images captured

            if (isVideo)
                if ( TheVideoCapturer.grab()==false) key=27;
        } while (key != 27 );
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}


void cvTackBarEvents(int pos, void*)
{
    (void)(pos);


    setParamsFromGlobalVariables(MDetector);

    // recompute
    Fps.start();
    TheMarkers=MDetector.detect(TheInputImage);
    Fps.stop();
    // chekc the speed by calculating the mean speed of all iterations
    TheInputImage.copyTo(TheInputImageCopy);
    if (iShowAllCandidates) {
        auto candidates=MDetector.getCandidates();
        for(auto cand:candidates)
            Marker(cand,-1).draw(TheInputImageCopy, Scalar(255, 0, 255),1);
    }

    for (unsigned int i = 0; i < TheMarkers.size(); i++) {
        cout << TheMarkers[i] << endl;
        TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255),2);
    }

    // draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i = 0; i < TheMarkers.size(); i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
    cv::putText(TheInputImageCopy,"fps="+to_string(1./Fps.getAvrg() ),cv::Point(10,20),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),2);

    cv::imshow("in",  TheInputImageCopy );
    cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));
    if(showMennu)printMenuInfo();

}
