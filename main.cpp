#include "GEV_Wrapper.h"
#include <string>
#include <iostream>
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// カメラの順番を固定する配列
// カメラのシリアルIDを入力することで、
// 配列に記述した順番でカメラ映像を取得できる
const string cameraID[15] = {
	
	"18551299",
	"18464590",
	"18464423",
	"18551296",
	"18509956",

	"18509958",
	"18408232",
	"18509333",
	"18465007",
	"18509955",
	
	"18464424",
	"18464589",
	"18509340",
	"18464421",
	"18551294",
};

// カメラオブジェクトの初期化
GEV_Drive cam(cameraID);

std::string getTimeStamp()
{
	auto now = std::chrono::system_clock::now();
	auto now_c = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(localtime(&now_c), "%Y%m%d%H%M%S");
	return ss.str();
}

int main() {
	int result = 0;
	vector<string> filenames;
	try {
		// カメラの初期化
		cam.Init();
		// カメラ画像の取得開始
		cam.BeginAcquisition();
		bool lp_break = true;
		while (lp_break) {
			vector<cv::Mat> Frames;
			// 1フレーム分すべてのカメラから画像を取得する
			cam >> Frames;
			if (cv::waitKey(1) == 27)lp_break = false;
			if (cv::waitKey(22) == 'c') {
				string time = getTimeStamp();
				for (int i = 0; i < Frames.size(); i++) {
					string filename = "./image/" + to_string(i) + "-" + time + ".png";
					filenames.push_back(filename);
					cv::imwrite(filename, Frames[i]);
					cout << "saving: " << filename << endl;
				}
			}
			for (int i = 0; i < Frames.size(); i++)
				cv::resize(Frames[i], Frames[i], cv::Size(320, 240));
			ShowAquiredImages(Frames);
		}
		cv::FileStorage fs("imagelist"+getTimeStamp()+".xml", cv::FileStorage::WRITE);
		fs << "images" << "[";
		fs << "./pattern.png";
		for (int i = 0; i < filenames.size(); i++) {
			fs << filenames[i];
		}
		fs << "]";
		// カメラ画像の取得終了
		cam.EndAcquisition();
	}
	catch (Spinnaker::Exception& e) {
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	return result;
}