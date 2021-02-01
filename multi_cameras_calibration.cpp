#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/ccalib/multicalib.hpp>
#include <opencv2/ccalib/randpattern.hpp>

using namespace std;
using namespace cv;

const char* usage =
"\n ランダムパターンを用いたマルチカメラキャリブレーションのためのコマンドライン例 \n"
"   multi_cameras_calibration -nc 5 -pw 800 -ph 600 -ct 1 -fe 0 -nm 25 -v 0 multi_camera_omnidir.xml \n"
"\n"
" ファイルmulti_camera_omnidir.xmlはimagelist_creatorによって \n"
"imagelist_creator multi_camera_omnidir.xmlとして生成されます。\n"
" multi_camera_omnidir.xmlの最初のファイル名はパターンであり、残りは写真名であり、\n"
"写真名はcameraIdx-timestamp.*の形式でなければなりません。\n";

static void help()
{
    printf("\nこれはマルチカメラキャリブレーションのサンプルですが、今のところランダムパターンしかサポートしていません。\n"
		   "ピンホールカメラと全方位カメラの両方に対応しています。\n"
		   "全方位カメラについては、詳細は omnidir.hpp を参照してください。"
           "Usage: mutiCamCalib \n"
           "    -nc <num_camera> # カメラの台数 \n"
           "    -pw <pattern_width> # 実際のランダムパターンの幅 \n"
           "    -ph <pattern_height> # 実際のランダムパターンの高さ \n"
           "    -ct <camera_type> # カメラの種類, 0 はピンホールカメラで, 1 は全天球カメラ \n"
           "    -fe # 特徴抽出を表示するかどうか\n"
           "    -nm # 画像の最小一致数 \n"
		   "	-v # verbose infomationを表示するかどうか \n"
           "    input_data # パターンファイル名と写真名のリストを持つテキストファイルは imagelist_creator によって生成されます。 \n");
    printf("\n %s", usage);
}


int main(int argc, char** argv)
{
    float patternWidth = 0.0f, patternHeight = 0.0f;
    int nCamera = 0, nMiniMatches = 0, cameraType = 0;
    const char* outputFilename = "multi-camera-results.xml";
    const char* inputFilename = 0;
    int showFeatureExtraction = 0, verbose = 0;
    if (argc < 2)
    {
        help();
        return 1;
    }

    for (int i = 1; i < argc; ++i)
    {
        const char* s = argv[i];
        if (strcmp( s, "-nc") == 0)
        {
            if (sscanf( argv[++i], "%u", &nCamera) != 1 || nCamera <= 0)
            {
                return fprintf(stderr, "Invalid number of cameras \n"), -1;
            }
        }
        else if ( strcmp( s, "-pw" ) == 0 )
        {
            if (sscanf( argv[++i], "%f", &patternWidth) != 1 || patternWidth <=0 )
            {
                return fprintf(stderr, "Invalid pattern width \n"), -1;
            }
        }
        else if ( strcmp( s, "-ph" ) == 0 )
        {
            if (sscanf( argv[++i], "%f", &patternHeight) != 1 || patternHeight <=0 )
            {
                return fprintf(stderr, "Invalid pattern height \n"), -1;
            }
        }
        else if ( strcmp( s, "-ct" ) == 0 )
        {
            if (sscanf( argv[++i], "%u", &cameraType) != 1 || (cameraType !=0 && cameraType !=1 && cameraType !=2) )
            {
                return fprintf(stderr, "Invalid camera type, 0 for pinhole and 1 for omnidirectional \n"), -1;
            }
        }
        else if ( strcmp( s, "-fe" ) == 0 )
        {
            if (sscanf( argv[++i], "%u", &showFeatureExtraction) != 1 || (showFeatureExtraction !=1 && showFeatureExtraction !=0) )
            {
                return fprintf(stderr, "Not bool value, set to 0 or 1 \n"), -1;
            }
        }
        else if ( strcmp( s, "-nm" ) == 0 )
        {
            if (sscanf( argv[++i], "%u", &nMiniMatches) != 1 || nMiniMatches <=0 )
            {
                return fprintf(stderr, "Invalid number of minimal matches \n"), -1;
            }
        }
		else if ( strcmp( s, "-v" ) == 0 )
		{
			if (sscanf( argv[++i], "%u", &verbose) != 1 || (verbose !=1 && verbose !=0) )
			{
				return fprintf(stderr, "verbose is not bool value, set to 0 or 1 \n"), -1;
			}
		}
        else if( s[0] != '-')
        {
            inputFilename = s;
        }
        else
        {
            return fprintf( stderr, "Unknown option %s\n", s ), -1;
        }
    }

    // do multi-camera calibration
    multicalib::MultiCameraCalibration multiCalib(cameraType, nCamera, inputFilename, patternWidth, patternHeight, verbose, showFeatureExtraction, nMiniMatches);

    multiCalib.loadImages();
    multiCalib.initialize();
    multiCalib.optimizeExtrinsics();
    // the above three lines can be replaced by multiCalib.run();


	multiCalib.writeParameters(outputFilename);
}