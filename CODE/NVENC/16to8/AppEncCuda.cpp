/*
* Copyright 2017-2018 NVIDIA Corporation.  All rights reserved.
*
* Please refer to the NVIDIA end user license agreement (EULA) associated
* with this source code for terms and conditions that govern your use of
* this software. Any use, reproduction, disclosure, or distribution of
* this software and related documentation outside the terms of the EULA
* is strictly prohibited.
*
* This is main10
*/
#include <numeric>
#include <fstream>
#include <iostream>
#include <memory>
#include <cuda.h>
#include "NvEncoder/NvEncoderCuda.h"
#include "../Utils/Logger.h"
#include "../Utils/NvEncoderCLIOptions.h"
#include "../Utils/NvCodecUtils.h"
#include <opencv2/opencv.hpp>

#include <string>
#include <time.h>

#include <vfile.h>
#include <ometiff_file.h>
#include <btf_file.h>

#include <cmath>

/*----------------------------------my function START---------------------------------------------------*/
void grey_to_nv12(cv::Mat &high, cv::Mat &img_nv12){
	// 将8bit grey -> nv12存储，输出img_nv12
	cv::Mat R(788 * 1 / 2, 2048, CV_32FC1, cv::Scalar(128));
	cv::vconcat(high, R, img_nv12);
}

void Convert8(cv::Mat& temp) {
	cv::Mat SqrtTemp;
	cv::pow(temp+1, 0.5, SqrtTemp);
	temp = (SqrtTemp - 1) ;
}



std::vector<uint> read_dimension(char *szInFilePath) {
	OmeTiffFile omeTiff;
	ErrorCode err = omeTiff.open(szInFilePath);
	if (err != ERROR_SUCCESS)
	{
		std::cout << "input ome tiff file path error.\n";
		// ERROR
		return{ 0,0 };
	}

	const TIFF_SIZE& tifSize = omeTiff.getTiffSize();
	std::cout << szInFilePath << ":\nDimension [C, Z, T] = " << "[" << tifSize.c << ", " << tifSize.z << ", " << tifSize.t << "]\n";
	std::cout << "[Width, Height] = [" << tifSize.x << ", " << tifSize.y << "]\n";
	std::vector<uint> dimension;
	dimension.push_back(tifSize.c);
	dimension.push_back(tifSize.z);
	dimension.push_back(tifSize.t);
	dimension.push_back(tifSize.x);
	dimension.push_back(tifSize.y);
	return dimension;
}

/*----------------------------------my function END---------------------------------------------------*/

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();

void EncodeCuda(CUcontext cuContext, char *szInFilePath, int nWidth, int nHeight, NV_ENC_BUFFER_FORMAT eFormat,
	char *szOutFilePath, char *szOutFilePath2, NvEncoderInitParam *pEncodeCLIOptions, NvEncoderInitParam *pEncodeCLIOptions2)
{
	OmeTiffFile omeTiff;
	ErrorCode err = omeTiff.open(szInFilePath);

	// ometiff： Read dimensions from Raw.ome.tiff
	std::vector<uint> dimension = read_dimension(szInFilePath);



	//for (uint i = 0; i < dimension[1]; i++) {
	for (uint i = 1; i < 2; i++) {
		// initialization for out path
		char OutFilePath[25];
		std::string path1 = szOutFilePath;
		sprintf(OutFilePath, "\\16to8_%05d.mkv", i);
		std::ofstream fpOut(path1+std::string(OutFilePath), std::ios::out | std::ios::binary);
		const clock_t begin_time9 = clock();

		// initialization for encoder
		NvEncoderCuda enc(cuContext, nWidth, nHeight, eFormat);


		NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };                                                  // log1:check?The name is very close. NO
		NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };                                                                            // log2:check?The name is also close. NO
		initializeParams.encodeConfig = &encodeConfig;																				   // log3:Maybe it just can define "codec name" and "preset"
		enc.CreateDefaultEncoderParams(&initializeParams, pEncodeCLIOptions->GetEncodeGUID(), pEncodeCLIOptions->GetPresetGUID());     // 构造默认的参数

		pEncodeCLIOptions->SetInitParams(&initializeParams, eFormat);

		enc.CreateEncoder(&initializeParams);																							// 近乎 走完了NvEncoder.cpp全部的

		//----------------------------------------------------------我的部分-----------------------------------------------------------------------------------------------

		//---------------------------------------------------------END----------------------------------------------------------------------------------------------------



		int nFrameSize = 2048 * 788 * 3/2;

		std::unique_ptr<uint8_t[]> pHostFrame(new uint8_t[nFrameSize]);																	/*std::unique_ptr 一种智能指针，它也是通过指针的方式来管理对象资源，并且在 unique_ptr 的生命期结束后释放该资源。
																																		std::unique_ptr::operator[]     你就把它当成数组名， 只不过智能管理，解决浪费问题了
																																		参考网址：https://en.cppreference.com/w/cpp/memory/unique_ptr/operator_at
																																		开辟一个nFrameSize大的空间
																																		*/





		// Inilization for parameters
		char name1[6];


		const clock_t begin = clock();

		std::vector<double> t1;
		std::vector<double> t2;
		std::vector<double> t3;
		std::vector<double> t4;
		std::vector<double> t5;
		std::vector<double> t6;
		cv::Mat high(788, 2048, CV_8UC1);
		cv::Mat low(788, 2048, CV_8UC1);
		cv::Mat high_nv12, low_nv12;

		int nFrame = 0;

		for (uint j = 0; j < dimension[2] + 4; j++) {//dimension[2]+4
			cv::Mat temp; // Save certain frame from Raw.ome.tif
			if (j < dimension[2]) {
				// Read a frame from Raw.ome.tif
				const clock_t begin_time0 = clock();
				omeTiff.saveSlice(temp, i, j);
				temp.convertTo(temp, CV_32FC1);
				t1.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);
				// Is it empty?
				if (!temp.empty()) {
					// read one frame
					const clock_t begin_time3 = clock();
					cv::Mat tiff_after = temp.clone();
					t1.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);


					//t4.push_back(float(clock() - begin_time3) / CLOCKS_PER_SEC);
					// split into high and low
					const clock_t begin_time9 = clock();

					// grey to nv12
					Convert8(temp);
					grey_to_nv12(temp, low_nv12);

					low_nv12.convertTo(low_nv12, CV_8UC1);

					//memcpy
					const clock_t begin_time1 = clock();

					std::memcpy(pHostFrame.get(), low_nv12.datastart, 2048 * 788 * 3/2);
					//.push_back(float(clock() - begin_time1) / CLOCKS_PER_SEC);

				}
			}
			if (!temp.empty() || j > dimension[2]) {
				// For receiving encoded packets
				std::vector<std::vector<uint8_t>> vPacket;
				const clock_t begin_time2 = clock();
				//if (nRead == nFrameSize)                         // 图片占用硬盘字节数 是否等于 NV12算出来的图像大小 													// nRead == nFrameSize
				if (1)
				{

					// low8
					const clock_t begin_time1 = clock();
					const NvEncInputFrame* encoderInputFrame = enc.GetNextInputFrame();														// 这个函数用于获取下一个可用的输入缓冲区。
					NvEncoderCuda::CopyToDeviceFrame(cuContext, pHostFrame.get(), 0, (CUdeviceptr)encoderInputFrame->inputPtr,				// 将参数传递到CUDA， pHostFrame.get()传的是数组地址，类似“0x1f8cc20”
						(int)encoderInputFrame->pitch,
						enc.GetEncodeWidth(),
						enc.GetEncodeHeight(),
						CU_MEMORYTYPE_HOST,
						encoderInputFrame->bufferFormat,
						encoderInputFrame->chromaOffsets,
						encoderInputFrame->numChromaPlanes);
					enc.EncodeFrame(vPacket);


				}
				else
				{
					enc.EndEncode(vPacket);
				}
				nFrame += (int)vPacket.size();

				t3.push_back(float(clock() - begin_time2) / CLOCKS_PER_SEC);
				const clock_t begin_time00 = clock();
				// 输出编码到了第几帧
				std::cout << "j = " << j << std::endl;
				for (std::vector<uint8_t> &packet : vPacket)
				{
					// For each encoded packet
					fpOut.write(reinterpret_cast<char*>(packet.data()), packet.size());
				}


			}
		}
		std::cout << "Total time = " << float(clock() - begin) / CLOCKS_PER_SEC << std::endl;
		enc.DestroyEncoder();
		fpOut.close();

	


		std::cout << "Total frames encoded: " << nFrame << std::endl << "Saved in file " << szOutFilePath << std::endl;

		double sum_t1 = std::accumulate(std::begin(t1), std::end(t1), 0.0);
		double mean1 = sum_t1 / t1.size(); //均值
		std::cout << "t1 读的时间" << sum_t1 << std::endl;

		double sum_t_de = std::accumulate(std::begin(t2), std::end(t2), 0.0);
		double mean2 = sum_t_de / t2.size(); //均值
		std::cout << "t2 我的操作 " << sum_t_de - sum_t1 << std::endl;

		double sum_t_wr = std::accumulate(std::begin(t3), std::end(t3), 0.0);
		double mean3 = sum_t_wr / t3.size(); //均值
		std::cout << "t3 编码的时间" << sum_t_wr << std::endl;

		double sum_t2 = std::accumulate(std::begin(t4), std::end(t4), 0.0);
		double mean4 = sum_t2 / t4.size(); //均值
		std::cout << "t4 " << sum_t2 << std::endl;

		double sum_t5 = std::accumulate(std::begin(t5), std::end(t5), 0.0);
		double mean5 = sum_t5 / t5.size(); //均值
		std::cout << "t5 " << sum_t5 << std::endl;

		double sum_t6 = std::accumulate(std::begin(t6), std::end(t6), 0.0);
		double mean6 = sum_t6 / t5.size(); //均值
		std::cout << "t5 " << sum_t6 << std::endl;
	}
}


void ShowEncoderCapability()
{
	ck(cuInit(0));
	int nGpu = 0;
	ck(cuDeviceGetCount(&nGpu));
	printf("Encoder Capability\n");
	printf("#  %-20.20s H264 H264_444 H264_ME H264_WxH  HEVC HEVC_Main10 HEVC_Lossless HEVC_SAO HEVC_444 HEVC_ME HEVC_WxH\n", "GPU");
	for (int iGpu = 0; iGpu < nGpu; iGpu++) {
		CUdevice cuDevice = 0;
		ck(cuDeviceGet(&cuDevice, iGpu));
		char szDeviceName[80];
		ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
		CUcontext cuContext = NULL;
		ck(cuCtxCreate(&cuContext, 0, cuDevice));
		NvEncoderCuda enc(cuContext, 1280, 720, NV_ENC_BUFFER_FORMAT_NV12);

		//Adjusted # %-20.20s H264  H264_444  H264_ME  H264_WxH HEVC  HEVC_Main10  HEVC_Lossless  HEVC_SAO  HEVC_444  HEVC_ME  HEVC_WxH
		printf("%-2d %-20.20s   %s      %s       %s    %4dx%-4d   %s       %s            %s           %s        %s       %s    %4dx%-4d\n",
			iGpu, szDeviceName,
			enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID, NV_ENC_CAPS_SUPPORTED_RATECONTROL_MODES) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID, NV_ENC_CAPS_SUPPORT_YUV444_ENCODE) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID, NV_ENC_CAPS_SUPPORT_MEONLY_MODE) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID, NV_ENC_CAPS_WIDTH_MAX),
			enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID, NV_ENC_CAPS_HEIGHT_MAX),
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_SUPPORTED_RATECONTROL_MODES) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_SUPPORT_10BIT_ENCODE) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_SUPPORT_LOSSLESS_ENCODE) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_SUPPORT_SAO) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_SUPPORT_YUV444_ENCODE) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_SUPPORT_MEONLY_MODE) ? "+" : "-",
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_WIDTH_MAX),
			enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID, NV_ENC_CAPS_HEIGHT_MAX)
			);

		enc.DestroyEncoder();
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
		<< "-i           Input file path" << std::endl
		<< "-o           Output file path" << std::endl
		<< "-s           Input resolution in this form: WxH" << std::endl
		<< "-if          Input format: iyuv nv12 yuv444 p010 yuv444p16 bgra bgra10 ayuv abgr abgr10" << std::endl
		<< "-gpu         Ordinal of GPU to use" << std::endl
		;
	oss << NvEncoderInitParam().GetHelpMessage() << std::endl;
	if (bThrowError)
	{
		throw std::invalid_argument(oss.str());
	}
	else
	{
		std::cout << oss.str();
		ShowEncoderCapability();
		exit(0);
	}
}

void ParseCommandLine(int argc, char *argv[], char *szInputFileName, int &nWidth, int &nHeight,
	NV_ENC_BUFFER_FORMAT &eFormat, char *szOutputFileName, char* szOutputFileName2, NvEncoderInitParam &initParam, int &iGpu)
{
	std::ostringstream oss;
	int i;
	for (i = 1; i < argc; i++)
	{
		if (!_stricmp(argv[i], "-h"))
		{
			ShowHelpAndExit();
		}
		if (!_stricmp(argv[i], "-i"))
		{
			if (++i == argc)
			{
				ShowHelpAndExit("-i");
			}
			sprintf(szInputFileName, "%s", argv[i]);
			continue;
		}
		if (!_stricmp(argv[i], "-o"))
		{
			if (++i == argc)
			{
				ShowHelpAndExit("-o");
			}
			sprintf(szOutputFileName, "%s", argv[i]);
			continue;
		}
		if (!_stricmp(argv[i], "-o2"))
		{
			if (++i == argc)
			{
				ShowHelpAndExit("-o");
			}
			sprintf(szOutputFileName2, "%s", argv[i]);
			continue;
		}

		if (!_stricmp(argv[i], "-s"))
		{
			if (++i == argc || 2 != sscanf(argv[i], "%dx%d", &nWidth, &nHeight))
			{
				ShowHelpAndExit("-s");
			}
			continue;
		}
		std::vector<std::string> vszFileFormatName =
		{
			"iyuv", "nv12", "yv12", "yuv444", "p010", "yuv444p16", "bgra", "bgra10", "ayuv", "abgr", "abgr10"
		};
		NV_ENC_BUFFER_FORMAT aFormat[] =
		{
			NV_ENC_BUFFER_FORMAT_IYUV,
			NV_ENC_BUFFER_FORMAT_NV12,
			NV_ENC_BUFFER_FORMAT_YV12,
			NV_ENC_BUFFER_FORMAT_YUV444,
			NV_ENC_BUFFER_FORMAT_YUV420_10BIT,
			NV_ENC_BUFFER_FORMAT_YUV444_10BIT,
			NV_ENC_BUFFER_FORMAT_ARGB,
			NV_ENC_BUFFER_FORMAT_ARGB10,
			NV_ENC_BUFFER_FORMAT_AYUV,
			NV_ENC_BUFFER_FORMAT_ABGR,
			NV_ENC_BUFFER_FORMAT_ABGR10,
		};
		if (!_stricmp(argv[i], "-if"))
		{
			if (++i == argc) {
				ShowHelpAndExit("-if");
			}
			auto it = std::find(vszFileFormatName.begin(), vszFileFormatName.end(), argv[i]);
			if (it == vszFileFormatName.end())
			{
				ShowHelpAndExit("-if");
			}
			eFormat = aFormat[it - vszFileFormatName.begin()];
			continue;
		}
		if (!_stricmp(argv[i], "-gpu"))
		{
			if (++i == argc)
			{
				ShowHelpAndExit("-gpu");
			}
			iGpu = atoi(argv[i]);
			continue;
		}
		// Regard as encoder parameter
		if (argv[i][0] != '-')
		{
			ShowHelpAndExit(argv[i]);
		}
		oss << argv[i] << " ";
		while (i + 1 < argc && argv[i + 1][0] != '-')
		{
			oss << argv[++i] << " ";
		}
	}
	initParam = NvEncoderInitParam(oss.str().c_str());
}

/**
*  This sample application illustrates encoding of frames in CUDA device buffers.
*  The application reads the image data from file and loads it to CUDA input
*  buffers obtained from the encoder using NvEncoder::GetNextInputFrame().
*  The encoder subsequently maps the CUDA buffers for encoder using NvEncodeAPI
*  and submits them to NVENC hardware for encoding as part of EncodeFrame() function.
*/




int main(int argc, char **argv) //这个意思就是cmd 输入
{
	char szInFilePath[256] = "C:\\Users\\USER\\Desktop\\PSNR\\Raw_tiff",
		szOutFilePath[256] = "",
		szOutFilePath2[256] = "";

	std::vector<double> t1;
	

	int nWidth = 2048, nHeight = 788;
	NV_ENC_BUFFER_FORMAT eFormat = NV_ENC_BUFFER_FORMAT_YUV420_10BIT;
	int iGpu = 0;
	try
	{
		NvEncoderInitParam encodeCLIOptions;

		ParseCommandLine(argc, argv, szInFilePath, nWidth, nHeight, eFormat, szOutFilePath, szOutFilePath2, encodeCLIOptions, iGpu); // 在这命令进行初始化的

		NvEncoderInitParam encodeCLIOptions2 = encodeCLIOptions;
		encodeCLIOptions2 = NvEncoderInitParam("-codec hevc -preset lossless_hp -profile main");
		//CheckInputFile(szInFilePath);

		if (!*szOutFilePath)
		{
			sprintf(szOutFilePath, encodeCLIOptions.IsCodecH264() ? "out.h264" : "out.hevc");
		}

		ck(cuInit(0));
		int nGpu = 0;
		ck(cuDeviceGetCount(&nGpu));
		if (iGpu < 0 || iGpu >= nGpu)
		{
			std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", " << nGpu - 1 << "]" << std::endl;
			return 1;
		}
		CUdevice cuDevice = 0;
		ck(cuDeviceGet(&cuDevice, iGpu));
		char szDeviceName[80];
		ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
		std::cout << "GPU in use: " << szDeviceName << std::endl;
		CUcontext cuContext = NULL;
		CUcontext cuContext2 = NULL;
		ck(cuCtxCreate(&cuContext, 0, cuDevice));
		ck(cuCtxCreate(&cuContext2, 2, cuDevice));


		const clock_t begin_time0 = clock();

		EncodeCuda(cuContext, szInFilePath, nWidth, nHeight, eFormat, szOutFilePath, szOutFilePath2, &encodeCLIOptions, &encodeCLIOptions2);
		t1.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);
		double sum_t1 = std::accumulate(std::begin(t1), std::end(t1), 0.0);
		double mean1 = sum_t1 / t1.size(); //均值
		std::cout << "t1adsfasf " << sum_t1 << std::endl;
		
	
}
	catch (const std::exception &ex)
	{
		std::cout << ex.what();
		return 1;
	}
	return 0;
}
