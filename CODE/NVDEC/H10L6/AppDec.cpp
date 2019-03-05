/*
* Copyright 2017-2018 NVIDIA Corporation.  All rights reserved.
*
* Please refer to the NVIDIA end user license agreement (EULA) associated
* with this source code for terms and conditions that govern your use of
* this software. Any use, reproduction, disclosure, or distribution of
* this software and related documentation outside the terms of the EULA
* is strictly prohibited.
*
*/

#include<numeric>
#include <iostream>
#include <algorithm>
#include <thread>
#include <cuda.h>
#include "NvDecoder/NvDecoder.h"
#include "../Utils/NvCodecUtils.h"
#include "../Utils/FFmpegDemuxer.h"
#include<opencv2/opencv.hpp>

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();


void ConvertToPlanar(uint8_t *pHostFrame, int nWidth, int nHeight, int nBitDepth) {
    if (nBitDepth == 8) {
        // nv12->iyuv
        YuvConverter<uint8_t> converter8(nWidth, nHeight);
        converter8.UVInterleavedToPlanar(pHostFrame);
    } else {
        // p016->yuv420p16
        YuvConverter<uint16_t> converter16(nWidth, nHeight);
        converter16.UVInterleavedToPlanar((uint16_t *)pHostFrame);
    }
}



void DecodeMediaFile(CUcontext cuContext, const char *szInFilePath, const char * szInFilePath2, const char *szOutFilePath, const char* szOutFilePath2, bool bOutPlanar,
	const Rect &cropRect, const Dim &resizeDim)
{
	//std::ofstream fpout(szInFilePath, std::ios::out | std::ios::binary);
	//if (!fpout)
	//{
	//	std::ostringstream err;
	//	err << "unable to open input file: " << szInFilePath << std::endl;
	//	throw std::invalid_argument(err.str());

	//}
	//char* path_h = std::string(szInFilePath) + "\\high.mkv";

	// demuxer - high 
	// demuxer2 - low 

	FFmpegDemuxer demuxer(szInFilePath);
	FFmpegDemuxer demuxer2(szInFilePath2);
	NvDecoder dec(cuContext, demuxer.GetWidth(), demuxer.GetHeight(), false, FFmpeg2NvCodecId(demuxer.GetVideoCodec()), NULL, false, false, &cropRect, &resizeDim);
	NvDecoder dec2(cuContext, demuxer2.GetWidth(), demuxer2.GetHeight(), false, FFmpeg2NvCodecId(demuxer2.GetVideoCodec()), NULL, false, false, &cropRect, &resizeDim);

	int nVideoBytes = 0, nVideoBytes2 = 0, nFrameReturned = 0, nFrameReturned2 = 0, nFrame = 0, j = 0;
	std::vector<cv::Mat> vbuffer;
	uint8_t *pVideo = NULL, **ppFrame;
	uint8_t *pVideo2 = NULL, **ppFrame2;
	std::vector<double> t_de;
	std::vector<double> t1;
	std::vector<double> t_wr;
	std::vector<double> t2;
	std::vector<double> t3;
	std::vector<double> t4;

	cv::Mat buffer = cv::Mat::zeros(788 * 3 / 2, 2048, CV_16UC1);
	cv::Mat buffer2 = cv::Mat::zeros(788 * 3 / 2, 2048, CV_8UC1);
	cv::Mat low8;


	std::string type(".tiff");
	char name1[6];

	const clock_t begin_time0 = clock(); // NO.0

	do {
		const clock_t begin_time_wr = clock();
		 // NO.de
		// 解复用
		demuxer.Demux(&pVideo, &nVideoBytes);
		demuxer2.Demux(&pVideo2, &nVideoBytes2);
		t_wr.push_back(float(clock() - begin_time_wr) / CLOCKS_PER_SEC);
		const clock_t begin_time1 = clock(); // NO.1
		// 解码
		dec.Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
		dec2.Decode(pVideo2, nVideoBytes2, &ppFrame2, &nFrameReturned2);
		t1.push_back(float(clock() - begin_time1) / CLOCKS_PER_SEC);


		const clock_t begin_time3 = clock();
		if (!nFrame && nFrameReturned)
			LOG(INFO) << dec.GetVideoInfo();

		for (int i = 0; i < nFrameReturned; i++) {
			if (bOutPlanar) {
				ConvertToPlanar(ppFrame[i], dec.GetWidth(), dec.GetHeight(), dec.GetBitDepth());
				ConvertToPlanar(ppFrame2[i], dec2.GetWidth(), dec2.GetHeight(), dec2.GetBitDepth());
			}

			
			//fpOut.write(reinterpret_cast<char*>(ppFrame[i]), dec.GetFrameSize());// NO.wr
		t3.push_back(float(clock() - begin_time3) / CLOCKS_PER_SEC);
	


			const clock_t begin_time_de = clock();
			cv::Mat merge16 = cv::Mat::zeros(788, 2048, CV_16UC1);
			memcpy(buffer.data, ppFrame[i], dec.GetFrameSize());
			memcpy(buffer2.data, ppFrame2[i], dec2.GetFrameSize());

			// high 10
			
			cv::Mat roi(buffer, cv::Rect(0, 0, 2048, 788));
			//roi.convertTo(roi, CV_32F);
			

			// low 6
			cv::cvtColor(buffer2, low8, 106);
			low8.convertTo(low8, CV_16UC1);
		

			t_de.push_back(float(clock() - begin_time_de) / CLOCKS_PER_SEC);
			const clock_t begin_time2 = clock(); // NO.2
			sprintf(name1, "%05d", j);
			//std::cout << bgra.type() << std::endl;
			std::string path = std::string(szOutFilePath) + "\\" + std::string(name1) + ".tiff";
			//std::string path2 = std::string(szOutFilePath2) + "\\" + std::string(name1) + ".tiff";
			//cv::imwrite(path, grey);
			//cv::imwrite(path2, grey2);
			merge16 = low8 + roi;
			cv::imwrite(path, merge16);
			t2.push_back(float(clock() - begin_time2) / CLOCKS_PER_SEC);
			j++;
			
		}
		nFrame += nFrameReturned;
		
		std::cout << "nFrame = " << nFrame << std::endl;
	} while (nVideoBytes);

	std::cout << "Total time = " << float(clock() - begin_time0) / CLOCKS_PER_SEC << std::endl;
	std::cout << "Total frame decoded: " << nFrame << std::endl
		<< "Saved in file " << szOutFilePath << std::endl;
	//	<< (dec.GetBitDepth() == 8 ? (bOutPlanar ? "iyuv" : "nv12") : (bOutPlanar ? "yuv420p16" : "p016"))
	//	<< " format" << std::endl;
	//std::cout << "vbuffer size = " << vbuffer.size() << std::endl;


	double sum_t1 = std::accumulate(std::begin(t1), std::end(t1), 0.0);

	std::cout << "解码时间 " << sum_t1 << std::endl;

	double sum_t_de = std::accumulate(std::begin(t_de), std::end(t_de), 0.0);

	std::cout << "我的操作 " << sum_t_de << std::endl;

	double sum_t_wr = std::accumulate(std::begin(t_wr), std::end(t_wr), 0.0);

	std::cout << "t_wr " << sum_t_wr << std::endl;

	double sum_t2 = std::accumulate(std::begin(t2), std::end(t2), 0.0);

	std::cout << "写的时间" << sum_t2 << std::endl;

	double sum_t3 = std::accumulate(std::begin(t3), std::end(t3), 0.0);

	std::cout << "t3 " << sum_t3 << std::endl;
	// 我的写入

	


}

void ShowDecoderCapability() {
    ck(cuInit(0));
    int nGpu = 0;
    ck(cuDeviceGetCount(&nGpu));
    std::cout << "Decoder Capability" << std::endl << std::endl;
    const char *aszCodecName[] = {"JPEG", "MPEG1", "MPEG2", "MPEG4", "H264", "HEVC", "HEVC", "HEVC", "VC1", "VP8", "VP9", "VP9", "VP9"};
    cudaVideoCodec aeCodec[] = { cudaVideoCodec_JPEG, cudaVideoCodec_MPEG1, cudaVideoCodec_MPEG2, cudaVideoCodec_MPEG4,
        cudaVideoCodec_H264, cudaVideoCodec_HEVC, cudaVideoCodec_HEVC, cudaVideoCodec_HEVC, cudaVideoCodec_VC1,
        cudaVideoCodec_VP8, cudaVideoCodec_VP9, cudaVideoCodec_VP9, cudaVideoCodec_VP9 };
    int anBitDepthMinus8[] = {0, 0, 0, 0, 0, 0, 2, 4, 0, 0, 0, 2, 4};
    for (int iGpu = 0; iGpu < nGpu; iGpu++) {
        CUdevice cuDevice = 0;
        ck(cuDeviceGet(&cuDevice, iGpu));
        char szDeviceName[80];
        ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
        CUcontext cuContext = NULL;
        ck(cuCtxCreate(&cuContext, 0, cuDevice));

        std::cout << "GPU " << iGpu << " - " << szDeviceName << std::endl << std::endl;
        for (int i = 0; i < sizeof(aeCodec) / sizeof(aeCodec[0]); i++) {
            CUVIDDECODECAPS decodeCaps = {};
            decodeCaps.eCodecType = aeCodec[i];
            decodeCaps.eChromaFormat = cudaVideoChromaFormat_420;
            decodeCaps.nBitDepthMinus8 = anBitDepthMinus8[i];

            cuvidGetDecoderCaps(&decodeCaps);
            std::cout << "Codec" << "  " << aszCodecName[i] << '\t' <<
                "BitDepth" << "  " << decodeCaps.nBitDepthMinus8 + 8 << '\t' <<
                "Supported" << "  " << (int)decodeCaps.bIsSupported << '\t' <<
                "MaxWidth" << "  " << decodeCaps.nMaxWidth << '\t' <<
                "MaxHeight" << "  " << decodeCaps.nMaxHeight << '\t' <<
                "MaxMBCount" << "  " << decodeCaps.nMaxMBCount << '\t' <<
                "MinWidth" << "  " << decodeCaps.nMinWidth << '\t' <<
                "MinHeight" << "  " << decodeCaps.nMinHeight << std::endl;
        }

        std::cout << std::endl;

        ck(cuCtxDestroy(cuContext));
    }
}

void ShowHelpAndExit(const char *szBadOption = NULL)
{
    bool bThrowError = false;
    std::ostringstream oss;
    if (szBadOption) 
    {
        bThrowError = true;
        oss << "Error parsing \"" << szBadOption << "\"" << std::endl;
    }
    oss << "Options:" << std::endl
        << "-i             Input file path" << std::endl
        << "-o             Output file path" << std::endl
        << "-outplanar     Convert output to planar format" << std::endl
        << "-gpu           Ordinal of GPU to use" << std::endl
        << "-crop l,t,r,b  Crop rectangle in left,top,right,bottom (ignored for case 0)" << std::endl
        << "-resize WxH    Resize to dimension W times H (ignored for case 0)" << std::endl
        ;
    oss << std::endl;
    if (bThrowError)
    {
        throw std::invalid_argument(oss.str());
    }
    else
    {
        std::cout << oss.str();
        ShowDecoderCapability();
        exit(0);
    }
}

void ParseCommandLine(int argc, char *argv[], char *szInputFileName, char* szInputFileName2, char *szOutputFileName, char* szOutputFileName2,
    bool &bOutPlanar, int &iGpu, Rect &cropRect, Dim &resizeDim)
{
    std::ostringstream oss;
    int i;
    for (i = 1; i < argc; i++) {
        if (!_stricmp(argv[i], "-h")) {
            ShowHelpAndExit();
        }
        if (!_stricmp(argv[i], "-i")) {
            if (++i == argc) {
                ShowHelpAndExit("-i");
            }
            sprintf(szInputFileName, "%s", argv[i]);
            continue;
        }
		if (!_stricmp(argv[i], "-i2")) {
			if (++i == argc) {
				ShowHelpAndExit("-i");
			}
			sprintf(szInputFileName2, "%s", argv[i]);
			continue;
		}
        if (!_stricmp(argv[i], "-o")) {
            if (++i == argc) {
                ShowHelpAndExit("-o");
            }
            sprintf(szOutputFileName, "%s", argv[i]);
            continue;
        }
		if (!_stricmp(argv[i], "-o2")) {
			if (++i == argc) {
				ShowHelpAndExit("-o");
			}
			sprintf(szOutputFileName2, "%s", argv[i]);
			continue;
		}
        if (!_stricmp(argv[i], "-outplanar")) {
            bOutPlanar = true;
            continue;
        }
        if (!_stricmp(argv[i], "-gpu")) {
            if (++i == argc) {
                ShowHelpAndExit("-gpu");
            }
            iGpu = atoi(argv[i]);
            continue;
        }
        if (!_stricmp(argv[i], "-crop")) {
            if (++i == argc || 4 != sscanf(
                    argv[i], "%d,%d,%d,%d",
                    &cropRect.l, &cropRect.t, &cropRect.r, &cropRect.b)) {
                ShowHelpAndExit("-crop");
            }
            if ((cropRect.r - cropRect.l) % 2 == 1 || (cropRect.b - cropRect.t) % 2 == 1) {
                std::cout << "Cropping rect must have width and height of even numbers" << std::endl;
                exit(1);
            }
            continue;
        }
        if (!_stricmp(argv[i], "-resize")) {
            if (++i == argc || 2 != sscanf(argv[i], "%dx%d", &resizeDim.w, &resizeDim.h)) {
                ShowHelpAndExit("-resize");
            }
            if (resizeDim.w % 2 == 1 || resizeDim.h % 2 == 1) {
                std::cout << "Resizing rect must have width and height of even numbers" << std::endl;
                exit(1);
            }
            continue;
        }
        ShowHelpAndExit(argv[i]);
    }
}

/**
*  This sample application illustrates the demuxing and decoding of media file with
*  resize and crop of the output image. The application supports both planar (YUV420P and YUV420P16)
*  and non-planar (NV12 and P016) output formats.
*/

int main(int argc, char **argv) 
{
	// szOutFilePath szOutFilePath high
	// szInputFileName2 szOutFilePath2 low
	char szInFilePath[256] = "", szOutFilePath[256] = "", szInFilePath2[256] = "", szOutFilePath2[256] = "";
    bool bOutPlanar = false;
    int iGpu = 0;
    Rect cropRect = {};
    Dim resizeDim = {};
    try
    {
		ParseCommandLine(argc, argv, szInFilePath, szInFilePath2, szOutFilePath, szOutFilePath2, bOutPlanar, iGpu, cropRect, resizeDim);
        //CheckInputFile(szInFilePath);

        //if (!*szOutFilePath) {
        //    sprintf(szOutFilePath, bOutPlanar ? "out.planar" : "out.native");
        //}

        ck(cuInit(0));
        int nGpu = 0;
        ck(cuDeviceGetCount(&nGpu));
        if (iGpu < 0 || iGpu >= nGpu) {
            std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", " << nGpu - 1 << "]" << std::endl;
            return 1;
        }
        CUdevice cuDevice = 0;
        ck(cuDeviceGet(&cuDevice, iGpu));
        char szDeviceName[80];
        ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
        std::cout << "GPU in use: " << szDeviceName << std::endl;
        CUcontext cuContext = NULL;
        ck(cuCtxCreate(&cuContext, 0, cuDevice));

        std::cout << "Decode with demuxing." << std::endl;
		DecodeMediaFile(cuContext, szInFilePath, szInFilePath2, szOutFilePath, szOutFilePath2, bOutPlanar, cropRect, resizeDim);
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what();
        exit(1);
    }

    return 0;
}
