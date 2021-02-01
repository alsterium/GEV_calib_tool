//
//　カメラキャリブレーションプログラムのサンプル(opencv4.0.0)
//
//	***使い方***
//
//  引数1：チェスボードのコーナー数の行
//　引数2：チェスボードのコーナー数の列
//　引数3：取り込みに成功したチェスボードの数の設定
//　引数4：ディレイ（？）
//　引数5：findChessboardCorners()に渡す画像のスケーリング
//	
//  (Ex. 10x7チェスボードを用いてキャリブレーションを行う場合(14枚の画像を用いてキャリブレーション)
//       ~.exe 10 7 14
//

#include <opencv2\opencv.hpp>
#include <iostream>
#include <opencv2\core\ocl.hpp>
#include "GEV_Wrapper.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

CameraList camList;

const string cameraID[1] = { "18464421" };

void help(char *argv[]) {

}



int main(int argc, char *argv[])
{
	int n_boards = 0;//入力リストにより設定される
	float image_sf = 1.0f;
	float delay = 1.f;
	int board_w = 0;
	int board_h = 0;

	if (argc < 4 || argc >6) {
		cout << "\nERROR:Wrong number of input parameters";
		help(argv);
		return -1;
	}
	board_w = atoi(argv[1]);
	board_h = atoi(argv[2]);
	n_boards = atoi(argv[3]);
	if (argc > 4)delay = atof(argv[4]);
	if (argc > 5)image_sf = atof(argv[5]);

	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);

	////OpenCLによるremap並列化
	////frame by frameでピクセルを再配置するので、cpuリソースを使いすぎる
	////OpenCV標準ライブラリでOpenCLサポートされている関数であれば自動的に並列化可能
	////

	////OpenCLがこのシステムで使えるかチェック
	//if (!cv::ocl::haveOpenCL()) {
	//	cout << "OpenCL not available...";
	//	return -1;
	//}

	////OpenCLを有効化
	////
	//cv::ocl::setUseOpenCL(true);

	////コンテクスト宣言
	//cv::ocl::Context context;
	//if (!context.create(cv::ocl::Device::TYPE_GPU)) {
	//	cout << "Context creation faild";
	//	return -1;
	//}
	////デバイスの列挙
	//cout << context.ndevices() << "GPU device(s) detected" << endl;
	//for (size_t i = 0; i < context.ndevices(); i++) {
	//	cv::ocl::Device device = context.device(i);
	//	cout << "-- Device " << i << "---" << endl;
	//	cout << "Name: " << device.name() << endl;
	//	cout << "Availability: " << device.available() << endl;
	//	cout << "Image Support: " << device.imageSupport() << endl;
	//	cout << "OpenCL C version: " << device.OpenCL_C_Version() << endl;
	//}
	////デバイス0番を使用する
	//cv::ocl::Device(context.device(0));

	/****************
	* 初期化処理
	*****************/
	// システムオブジェクトのシングルトン参照を取得
	SystemPtr system = System::GetInstance();
	// カメラの初期化を行う
	InitCameras(system, camList, cameraID);
	/****************
	*****************/

	// カメラへのshared pointerを生成する
	CameraPtr pCam = nullptr;

	try
	{
		for (int i = 0; i < camList.GetSize(); i++) {
			pCam = camList.GetBySerial(cameraID[i]);
			pCam->BeginAcquisition();
		}
		
	}
	catch (const std::exception& e)
	{
		cout << "Error: " << e.what() << endl;
	}


	//格納場所を確保
	//
	vector<vector<cv::Point2f>>image_points;
	vector<vector<cv::Point3f>>object_points;

	//コーナーの画像を取り込む。取り込みに成功した（ボード上のすべての
	//コーナーが見つかった)画像がn_board枚得られるまでループする
	//
	double last_captured_timestamp = 0;
	cv::Size image_size;

	while (image_points.size() < (size_t)n_boards) {
		vector<cv::Mat> MImage;
		cv::Mat image0, image;
		//1frame読み込み
		vector<cv::Mat> Frames(AquireMultiCamImagesMT(camList, cameraID));
		image0 = Frames[0];
	
		image_size = image0.size();
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);//画像を1/2にリサイズ

		//ボードを探す
		//
		vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_sz, corners, cv::CALIB_CB_FAST_CHECK);
		//bool found = cv::findCirclesGrid(image, board_sz, corners, cv::CALIB_CB_ASYMMETRIC_GRID);

		//よいボードが見つかれば、データに加える
		//
		double timestamp = (double)clock() / CLOCKS_PER_SEC;

		if (found && timestamp - last_captured_timestamp > 1) {

			last_captured_timestamp = timestamp;
			image ^= cv::Scalar::all(255);//反転
			cv::drawChessboardCorners(image, board_sz, corners, found);

			cv::Mat mcorners(corners);													//データをコピーしない
			mcorners *= (1. / image_sf);												//コーナーの座標をスケーリングする
			image_points.push_back(corners);											//コーナーの座標をimage_pointsに追加
			object_points.push_back(vector<cv::Point3f>());								//空のvector<cv::Point3f>を追加(0.0,0.0,0.0)
			vector<cv::Point3f>& opts = object_points.back();							//object_pointsの末尾要素の参照opts
			opts.resize(board_n);														//ボードのコーナー数分vector<cv::Point3f>>を確保
			for (int j = 0; j < board_n; j++) {
				opts[j] = cv::Point3f((float)(j / board_w), (float)(j%board_w), 0.f);	//チェスボードの物体上の座標系
																						//(ex.j=1, board_w=10のとき、opts[1]=(0.0,1.0,0.0))
			}
			cout << "Collected our" << (int)image_points.size() <<
				"of" << n_boards << " needed chessboard images\n" << endl;
		}
		cv::imshow("Calibration", image);

		if ((cv::waitKey(30) & 255) == 27)
			return -1;
	}
	//画像の取り込みのループここまで

	cv::destroyWindow("Calibration");
	cout << "\n\n*** CALIBRATING THE CAMERA...\n" << endl;

	//カメラのキャリブレーションを行う！
	//

	//double init_intrinsic_matrix[]{ 554., 0., 320.,0., 582., 240., 0., 0., 1. };
	//cv::Mat intrinsic_matrix(3,3,CV_64F,init_intrinsic_matrix);<-逆に曲がる

	cv::Mat intrinsic_matrix;
	cv::Mat distortion_coeffs;
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
	);

	//内部パラメータと歪み係数を保持する
	cout << "***DONE!\n\nReprojection error is" << err <<
		"\nStoring Intrinsics.xml and Distortions.xml files\n\n";
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);

	fs << "image_width" << image_size.width << "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
		<< distortion_coeffs;

	fs.release();

	//これらの行列を読み込み直す例
	fs.open("intrinsics.xml", cv::FileStorage::READ);
	cout << "\nimage width: " << (int)fs["image_width"];
	cout << "\nimage height: " << (int)fs["image_height"];

	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	cout << "\nintrinsic matrix: " << intrinsic_matrix_loaded;
	cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;

	//後続フレーム全てに対して用いる歪み補正用のマップを作成する
	//
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		intrinsic_matrix_loaded,
		distortion_coeffs_loaded,
		cv::Mat(),
		intrinsic_matrix_loaded,
		image_size,
		CV_16SC2,
		map1,
		map2
	);

	//カメラ画像を画面に表示する
	//ここで、原画像と歪み補正後の画像を表示する
	//
	double start_t = 0;
	double end_t = 0;
	double sum_t = 0;
	int i = 0;
	cv::String frame_per_second;

	while (1) {
		start_t = (double)clock()/CLOCKS_PER_SEC;
		vector<cv::Mat> MImage(AquireMultiCamImagesMT(camList, cameraID));
		cv::Mat cl_image, image;
		cl_image = MImage[0];

		if (cl_image.empty())break;
		cv::remap(
			cl_image,
			image,
			map1,
			map2,
			cv::INTER_LINEAR,
			cv::BORDER_CONSTANT,
			cv::Scalar()
		);
		vector<cv::Point2f>corners;
		bool found = cv::findChessboardCorners(
			image,
			board_sz,
			corners,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK
		);
		cv::drawChessboardCorners(image, board_sz, corners, found);
		if ((cv::waitKey(1) & 255) == 27)break;
		end_t = (double)clock() / CLOCKS_PER_SEC;

		if (i > 20) {
			frame_per_second = std::to_string((int)(1.0 / (sum_t / (double)i)));
			frame_per_second += "fps";
			i = 0;
			sum_t = 0;
		}
		else {
			sum_t += end_t - start_t;
			i++;
		}
		cv::putText(image, frame_per_second, cv::Point(image.cols-100,image.rows-50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2, 8, false);
		cv::imshow("Undistorterd", image);
		//cv::imshow("distort", cl_image);
		
			
	}

	/**************
	* 終了処理
	***************/
	//カメラの初期化を解除
	pCam->DeInit();
	// カメラへのポインタを開放
	pCam = nullptr;
	// カメラの終了処理
	DeinitCameras(system, camList);
	/**************
	***************/




	return 0;
}

