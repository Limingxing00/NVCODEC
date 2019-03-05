/*
* Copyright 2017-2018 NVIDIA Corporation.  All rights reserved.
*
* Please refer to the NVIDIA end user license agreement (EULA) associated
* with this source code for terms and conditions that govern your use of
* this software. Any use, reproduction, disclosure, or distribution of
* this software and related documentation outside the terms of the EULA
* is strictly prohibited.
*

* this is high and low 8 bits.
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


/*----------------------------------my function START---------------------------------------------------*/
inline void grey2nv12_8(cv::Mat &high, cv::Mat &img_nv12){
	cv::Mat R(788 * 1 / 2, 2048, CV_8UC1, cv::Scalar(128));
	cv::vconcat(high, R, img_nv12);
}

inline void grey2nv12_10(cv::Mat &high, cv::Mat &img_nv12) {
	cv::Mat R(788 * 1 / 2, 2048, CV_16UC1, cv::Scalar(512));
	cv::vconcat(high, R, img_nv12);
}

inline void split_16bit(cv::Mat &tiff16, cv::Mat &tiff_after, cv::Mat &high8, cv::Mat &low8){
	

	// LOW 10
	const clock_t begin_time1 = clock();
	low8 = tiff_after & 0x3ff;
	//low8.convertTo(low8, CV_8UC1);

	// HIGH 6
	const clock_t begin_time2 = clock();
	tiff16 = tiff16 & 0xfc00;
	high8 = tiff16 / 1024;
	high8.convertTo(high8, CV_8UC1);
	//std::cout << "high " << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
	
}

std::vector<uint> read_dimension(char *szInFilePath){
	OmeTiffFile omeTiff;
	ErrorCode err = omeTiff.open(szInFilePath);
	if (err != ERROR_SUCCESS)
	{
		std::cout << "input ome tiff file path error.\n";
		// ERROR
		return{0,0};
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

std::vector<std::ofstream*> OutFilePathInfo(char* szOutFilePath, char* szOutFilePath2, int i) {
	char OutFilePath[25];
	char OutFilePath2[25];

	// high 8 bits
	std::string path1 = szOutFilePath;
	// low 8 bits 
	std::string path2 = szOutFilePath2;

	sprintf(OutFilePath, "\\H6L10_low%05d.mkv", i);
	sprintf(OutFilePath2, "\\H6L10_high%05d.mkv", i);
	std::ofstream* fpOut = new std::ofstream(path2 + OutFilePath, std::ios::out | std::ios::binary);
	std::ofstream* fpOut2 = new std::ofstream(path1 + OutFilePath2, std::ios::out | std::ios::binary);
	// vector of outpath , which the first is high 8 bits and the second is low 8 bits.
	std::vector<std::ofstream*> vfpOut;
	vfpOut.push_back(fpOut2);
	vfpOut.push_back(fpOut);
	return vfpOut;
}

inline void InitialEncoder(NvEncoderInitParam *pEncodeCLIOptions, NV_ENC_BUFFER_FORMAT eFormat, NvEncoderCuda& enc) {
	NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };                                                  // log1:check?The name is very close. NO
	NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };                                                                            // log2:check?The name is also close. NO
	initializeParams.encodeConfig = &encodeConfig;																				   // log3:Maybe it just can define "codec name" and "preset"
	enc.CreateDefaultEncoderParams(&initializeParams, pEncodeCLIOptions->GetEncodeGUID(), pEncodeCLIOptions->GetPresetGUID());     // 构造默认的参数

	pEncodeCLIOptions->SetInitParams(&initializeParams, eFormat);

	enc.CreateEncoder(&initializeParams);
}

inline void LeftSix(cv::Mat& temp) {

	temp = temp * 64;

}
/*----------------------------------my function END---------------------------------------------------*/

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();


void EncodeCuda(CUcontext cuContext, char *szInFilePath, int nWidth, int nHeight, NV_ENC_BUFFER_FORMAT eFormat,
	char *szOutFilePath, char *szOutFilePath2, NvEncoderInitParam *pEncodeCLIOptions, NvEncoderInitParam *pEncodeCLIOptions2)
{
	const clock_t begin_time9 = clock();
	OmeTiffFile omeTiff;
	ErrorCode err = omeTiff.open(szInFilePath);

	// ometiff： Read dimensions from Raw.ome.tiff
	std::vector<uint> dimension = read_dimension(szInFilePath);

	//for (uint i = 0; i < dimension[1]; i++) 
	for (uint i = 1; i < 2; i++) {
		// Initialization for out path
		std::vector<std::ofstream*> vfpOut = OutFilePathInfo(szOutFilePath, szOutFilePath2, i);
		NV_ENC_BUFFER_FORMAT eFormat_10 = NV_ENC_BUFFER_FORMAT_YUV420_10BIT;
		NV_ENC_BUFFER_FORMAT eFormat_8 = NV_ENC_BUFFER_FORMAT_NV12;
		//// Create encoder
		//// Initialize the encoder for low 8 bits 
		//NvEncoderCuda enc(cuContext, nWidth, nHeight, eFormat);
		//InitialEncoder(pEncodeCLIOptions, eFormat, enc);
		//// Initialize the encoder for high 8 bits 
		//NvEncoderCuda enc2(cuContext, nWidth, nHeight, eFormat);
		//InitialEncoder(pEncodeCLIOptions2, eFormat, enc2);

		// 低10位 lossy
		NvEncoderCuda enc(cuContext, nWidth, nHeight, eFormat_10);


		NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };                                                  // log1:check?The name is very close. NO
		NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };                                                                            // log2:check?The name is also close. NO
		initializeParams.encodeConfig = &encodeConfig;																				   // log3:Maybe it just can define "codec name" and "preset"
		enc.CreateDefaultEncoderParams(&initializeParams, pEncodeCLIOptions->GetEncodeGUID(), pEncodeCLIOptions->GetPresetGUID());     // 构造默认的参数

		pEncodeCLIOptions->SetInitParams(&initializeParams, eFormat_10);

		enc.CreateEncoder(&initializeParams);																							// 近乎 走完了NvEncoder.cpp全部的

		//----------------------------------------------------------我的部分-----------------------------------------------------------------------------------------------
		// 高6位 无损
		NvEncoderCuda enc2(cuContext, nWidth, nHeight, eFormat_8);


		NV_ENC_INITIALIZE_PARAMS initializeParams2 = { NV_ENC_INITIALIZE_PARAMS_VER };                                                  // log1:check?The name is very close. NO
		NV_ENC_CONFIG encodeConfig2 = { NV_ENC_CONFIG_VER };                                                                            // log2:check?The name is also close. NO
		initializeParams2.encodeConfig = &encodeConfig2;																				   // log3:Maybe it just can define "codec name" and "preset"
		enc2.CreateDefaultEncoderParams(&initializeParams2, pEncodeCLIOptions2->GetEncodeGUID(), pEncodeCLIOptions2->GetPresetGUID());     // 构造默认的参数

		pEncodeCLIOptions2->SetInitParams(&initializeParams2, eFormat_8);

		enc2.CreateEncoder(&initializeParams2);

		//---------------------------------------------------------END----------------------------------------------------------------------------------------------------

		// Inilization parameters
		int nFrameSize = enc.GetFrameSize();

		std::unique_ptr<uint8_t[]> pHostFrame(new uint8_t[nFrameSize]);																	/*std::unique_ptr 一种智能指针，它也是通过指针的方式来管理对象资源，并且在 unique_ptr 的生命期结束后释放该资源。
																																		std::unique_ptr::operator[]     你就把它当成数组名， 只不过智能管理，解决浪费问题了
																																		参考网址：https://en.cppreference.com/w/cpp/memory/unique_ptr/operator_at
																																		开辟一个nFrameSize大的空间
																																		*/
		std::unique_ptr<uint8_t[]> pHostFrame2(new uint8_t[nFrameSize]);

		const clock_t begin = clock();

		std::vector<double> t1;
		std::vector<double> t2;
		std::vector<double> t3;
		std::vector<double> t4;
		std::vector<double> t5;
		std::vector<double> t6;
		cv::Mat high(788, 2048, CV_8UC1);
		cv::Mat low;
		cv::Mat high_nv12, low_nv12;
		// 需要编码的帧数

		int nFrame = 0;
		for (uint j = 0; j < dimension[2] + 4; j++) {//dimension[2]+4
			cv::Mat temp; // Save certain frame from Raw.ome.tif
			if (j < dimension[2]) {
				// Read a frame from Raw.ome.tif
				omeTiff.saveSlice(temp, i, j);
				//char pp[50];
				//sprintf(pp, "%05d.tiff", j);
				//if (!temp.empty()) {
				//	cv::imwrite("C:\\Users\\USER\\Desktop\\test\\dataset1\\" + std::string(pp), temp);
				//}
				//

				//printf("i =", j);
				// Is it empty?
				if (!temp.empty())  {
					// encode per frame

					const clock_t begin_time0 = clock();

					// read one frame


					//data_read()
					const clock_t begin_time3 = clock();
					cv::Mat tiff_after = temp.clone();
					t1.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);

					//temp.convertTo(temp, CV_32F);

					const clock_t begin_time9 = clock();
					split_16bit(temp, tiff_after, high, low);
					
					// grey to nv12
					grey2nv12_8(high, high_nv12);
					LeftSix(low);
					grey2nv12_10(low, low_nv12);



					const clock_t begin_time1 = clock();
					std::memcpy(pHostFrame2.get(), high_nv12.datastart, 2048 * 788 * 3 / 2);
					std::memcpy(pHostFrame.get(), low_nv12.datastart, 2048 * 788 * 3 );

					t2.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);
					//t2.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);

					//std::cout << "Data transfer = " << float(clock() - begin_time0) / CLOCKS_PER_SEC << std::endl;

				}

			}
			// 编码
			if (!temp.empty() || j > dimension[2]) {
				// For receiving encoded packets
				std::vector<std::vector<uint8_t>> vPacket;
				std::vector<std::vector<uint8_t>> vPacket2;
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

					// high8
					const NvEncInputFrame* encoderInputFrame2 = enc2.GetNextInputFrame();
					NvEncoderCuda::CopyToDeviceFrame(cuContext, pHostFrame2.get(), 0, (CUdeviceptr)encoderInputFrame2->inputPtr,				// 将参数传递到CUDA， pHostFrame.get()传的是数组地址，类似“0x1f8cc20”
						(int)encoderInputFrame2->pitch,
						enc2.GetEncodeWidth(),
						enc2.GetEncodeHeight(),
						CU_MEMORYTYPE_HOST,
						encoderInputFrame2->bufferFormat,
						encoderInputFrame2->chromaOffsets,
						encoderInputFrame2->numChromaPlanes);
					enc2.EncodeFrame(vPacket2);

					std::cout << "j = " << j << std::endl;
					//std::cout << "Encoding one frame = " << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
					/*应用程序必须调用EncodeFrame()函数对未压缩的数据进行编码，
					这些数据已复制到从GetNextInputFrame()函数获得的输入缓冲区中。*/

				}
				else
				{
					enc.EndEncode(vPacket);
					enc2.EndEncode(vPacket2);

				}
				nFrame += (int)vPacket.size();

				t3.push_back(float(clock() - begin_time2) / CLOCKS_PER_SEC);
				const clock_t begin_time00 = clock();

				// low write
				for (std::vector<uint8_t> &packet : vPacket)
				{
					// For each encoded packet
					(*vfpOut[1]).write(reinterpret_cast<char*>(packet.data()), packet.size());
				}

				// high write
				for (std::vector<uint8_t> &packet2 : vPacket2)
				{
					// For each encoded packet
					(*vfpOut[0]).write(reinterpret_cast<char*>(packet2.data()), packet2.size());
				}

				//t4.push_back(float(clock() - begin_time2) / CLOCKS_PER_SEC);

				t6.push_back(float(clock() - begin_time00) / CLOCKS_PER_SEC);
				//if (name1 == "00816") break;


				// write into disk

				//destroy two encoders
			}
		}
		std::cout << "Total time = " << float(clock() - begin) / CLOCKS_PER_SEC << std::endl;
		enc.DestroyEncoder();
		enc2.DestroyEncoder();
		(*vfpOut[1]).close();
		(*vfpOut[0]).close();


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

void ParseCommandLine(int argc, char *argv[], char *szInputFileName, int &nWidth, int &nHeight, int& encoder_num,
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
	char szInFilePath[256] = "",
		szOutFilePath[256] = "",
		szOutFilePath2[256] = "";

	// the number of frames
	int encoder_num;

	

	int nWidth = 2048, nHeight = 788;
	NV_ENC_BUFFER_FORMAT eFormat = NV_ENC_BUFFER_FORMAT_IYUV;
	int iGpu = 0;
	try
	{
		NvEncoderInitParam encodeCLIOptions;

		ParseCommandLine(argc, argv, szInFilePath, nWidth, nHeight, encoder_num, eFormat, szOutFilePath, szOutFilePath2, encodeCLIOptions, iGpu); // 在这命令进行初始化的

		NvEncoderInitParam encodeCLIOptions2 = encodeCLIOptions;
		encodeCLIOptions2 = NvEncoderInitParam("-codec hevc -preset lossless -profile main");
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
	

}
	catch (const std::exception &ex)
	{
		std::cout << ex.what();
		return 1;
	}
	return 0;
}
