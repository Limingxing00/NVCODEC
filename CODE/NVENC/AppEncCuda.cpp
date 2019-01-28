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

/*----------------------------------my function START---------------------------------------------------*/
void grey_to_nv12(cv::Mat &high, cv::Mat &img_nv12){
	// ��8bit grey -> nv12�洢�����img_nv12
	cv::Mat R(788 * 1 / 2, 2048, CV_8UC1, cv::Scalar(128));
	cv::vconcat(high, R, img_nv12);
}

void split_16bit(cv::Mat &tiff16, cv::Mat &tiff_after, cv::Mat &high8, cv::Mat &low8){
	// low8
	cv::MatIterator_<ushort> grayit0, grayend0;

	

	const clock_t begin_time1 = clock();
	//for (grayit0 = tiff_after.begin<ushort>(), grayend0 = tiff_after.end<ushort>(); grayit0 != grayend0; grayit0++){
	//	*grayit0 = *grayit0 & 0xff;
	//}
	tiff_after = tiff_after & 0xff;

	//cv::bitwise_and(tiff16, 0xff, tiff_after);

	//std::cout << "low " << float(clock() - begin_time1) / CLOCKS_PER_SEC << std::endl;
	tiff_after.convertTo(low8, CV_8UC1);
	
	// high
	//cv::MatIterator_<ushort> grayit, grayend;
	//for (grayit = tiff16.begin<ushort>(), grayend = tiff16.end<ushort>(); grayit != grayend; grayit++){
	//	*grayit = *grayit >> 8;                                                                   // ����Ǹղ��Ǹ����صĵ�ַ�������ǻҶȣ�������һ��uchar
	//}
	const clock_t begin_time2 = clock();



	tiff16 = tiff16 /256-0.5;
	tiff16.convertTo(high8, CV_8UC1, 1, 0);
	//std::cout << "high " << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
	
}

/*----------------------------------my function END---------------------------------------------------*/

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();


void EncodeCuda(CUcontext cuContext, char *szInFilePath, int nWidth, int nHeight, int encoder_num, NV_ENC_BUFFER_FORMAT eFormat,
	char *szOutFilePath, char *szOutFilePath2, NvEncoderInitParam *pEncodeCLIOptions, NvEncoderInitParam *pEncodeCLIOptions2)
{
	//std::ifstream fpIn(szInFilePath, std::ifstream::in | std::ifstream::binary);
	//if (!fpIn)
	//{
	//	std::ostringstream err;
	//	err << "Unable to open input file: " << szInFilePath << std::endl;
	//	throw std::invalid_argument(err.str());
	//}

	std::string path1 = szOutFilePath;
	std::string path2 = szOutFilePath2;

	std::ofstream fpOut(path2, std::ios::out | std::ios::binary);


	std::ofstream fpOut2(path1, std::ios::out | std::ios::binary);
	if (!fpOut)
	{
		std::ostringstream err;
		err << "Unable to open output file: " << szOutFilePath << std::endl;
		throw std::invalid_argument(err.str());
	}
	const clock_t begin_time9 = clock();
	NvEncoderCuda enc(cuContext, nWidth, nHeight, eFormat);


	NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };                                                  // log1:check?The name is very close. NO
	NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };                                                                            // log2:check?The name is also close. NO
	initializeParams.encodeConfig = &encodeConfig;																				   // log3:Maybe it just can define "codec name" and "preset"
	enc.CreateDefaultEncoderParams(&initializeParams, pEncodeCLIOptions->GetEncodeGUID(), pEncodeCLIOptions->GetPresetGUID());     // ����Ĭ�ϵĲ���

	pEncodeCLIOptions->SetInitParams(&initializeParams, eFormat);

	enc.CreateEncoder(&initializeParams);																							// ���� ������NvEncoder.cppȫ����

	//----------------------------------------------------------�ҵĲ���-----------------------------------------------------------------------------------------------
	// �߰�λ ����
	NvEncoderCuda enc2(cuContext, nWidth, nHeight, eFormat);


	NV_ENC_INITIALIZE_PARAMS initializeParams2 = { NV_ENC_INITIALIZE_PARAMS_VER };                                                  // log1:check?The name is very close. NO
	NV_ENC_CONFIG encodeConfig2 = { NV_ENC_CONFIG_VER };                                                                            // log2:check?The name is also close. NO
	initializeParams2.encodeConfig = &encodeConfig2;																				   // log3:Maybe it just can define "codec name" and "preset"
	enc2.CreateDefaultEncoderParams(&initializeParams2, pEncodeCLIOptions2->GetEncodeGUID(), pEncodeCLIOptions2->GetPresetGUID());     // ����Ĭ�ϵĲ���

	pEncodeCLIOptions2->SetInitParams(&initializeParams2, eFormat);

	enc2.CreateEncoder(&initializeParams2);
	//---------------------------------------------------------END----------------------------------------------------------------------------------------------------



	int nFrameSize = enc.GetFrameSize();

	std::unique_ptr<uint8_t[]> pHostFrame(new uint8_t[nFrameSize]);																	/*std::unique_ptr һ������ָ�룬��Ҳ��ͨ��ָ��ķ�ʽ�����������Դ�������� unique_ptr �������ڽ������ͷŸ���Դ��
																																	std::unique_ptr::operator[]     ��Ͱ��������������� ֻ�������ܹ�������˷�������
																																	�ο���ַ��https://en.cppreference.com/w/cpp/memory/unique_ptr/operator_at
																																	����һ��nFrameSize��Ŀռ�
																																	*/
	std::unique_ptr<uint8_t[]> pHostFrame2(new uint8_t[nFrameSize]);





	// ��Ӧ�����⶯���ֽ�


	std::vector<cv::Mat> vbuffer(819);

	//std::string rootdir = "D:\\test\\raw\\low8";

	char name1[6];
	//cv::Mat R(788 * 1 / 2, 2048, CV_8UC1, cv::Scalar(128));


	////-----------------------------
	//for (int j = 0; j < 819; j++){
	//	// Read 4 images
	//	sprintf(name1, "%05d", j);

	//	cv::Mat imgs0 = cv::imread(rootdir + "\\" + std::string(name1) + ".tiff", 0);

	//	if (imgs0.empty()){
	//		std::cout << "InPut error: Please check the images paths." << std::endl;

	//	}

	//	// Get the size of pictures.
	//	// w : 2048
	//	// h : 788
	//	//int w = imgs0.cols;
	//	//int h = imgs0.rows;
	//	//std::cout << "The image size is " << w << " x " << h << std::endl;
	//	cv::Mat buffer;
	//	cv::vconcat(imgs0, R, buffer);

	//	//cv::cvtColor(imgs0, buffer, 106);
	//	vbuffer[j] = buffer;
	//	//std::cout << "j = " << j << std::endl;

	//}
	//std::cout << vbuffer.size() << std::endl;
	// question is above
	//cv::imshow("i = 20", vbuffer.at(20));
	//cv::waitKey();
	//cv::imshow("i = 155", vbuffer.at(155));
	//cv::waitKey();

	// check over
	int i = 0;

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
	// ��Ҫ�����֡��

	int nFrame = 0;
	while (true)
	{
		
		// Load the next frame from disk
		// std::streamsize nRead = fpIn.read(reinterpret_cast<char*>(pHostFrame.get()), nFrameSize).gcount();
		/*
		streamsize������һ�������ŵ��������ͣ����ڱ�ʾ��I / O�����д�����ַ�����I / O�������Ĵ�С��
		reinterpret_cast     ��ָʾ�����������ʽ����pHostFrame.get()��Ϊ����char*���͡�
		pHostFrame.get()     �������ʼ��ַ
		nFrameSize           Ҫ�����ֽ���������������������λ��
		gcount()             �������ַ���
		*/
		//std::cout << "i = " << i << std::endl;


		const clock_t begin_time0 = clock();
		if (i < encoder_num){
			
			// read one frame
			sprintf(name1, "%05d", i);
			cv::Mat imgs0 = cv::imread(std::string(szInFilePath) + "\\" + std::string(name1) + ".tiff", -1);
		
			const clock_t begin_time3 = clock();
			cv::Mat tiff_after = imgs0.clone();
			t1.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);
			
			imgs0.convertTo(imgs0, CV_32F);		
			//t4.push_back(float(clock() - begin_time3) / CLOCKS_PER_SEC);
			// split into high and low
			const clock_t begin_time9 = clock();
			split_16bit(imgs0, tiff_after, high, low);
		

			// grey to nv12
			
			grey_to_nv12(high, high_nv12);
			grey_to_nv12(low, low_nv12);
		

			//memcpy
			const clock_t begin_time1 = clock();
			std::memcpy(pHostFrame2.get(), high_nv12.datastart, 2048 * 788 * 3 / 2);
			std::memcpy(pHostFrame.get(), low_nv12.datastart, 2048 * 788 * 3 / 2);
			//.push_back(float(clock() - begin_time1) / CLOCKS_PER_SEC);
			
		}
		t2.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);
		//t2.push_back(float(clock() - begin_time0) / CLOCKS_PER_SEC);

		//std::cout << "Data transfer = " << float(clock() - begin_time0) / CLOCKS_PER_SEC << std::endl;




		// For receiving encoded packets
		std::vector<std::vector<uint8_t>> vPacket;
		std::vector<std::vector<uint8_t>> vPacket2;
		const clock_t begin_time2 = clock();
		//if (nRead == nFrameSize)                         // ͼƬռ��Ӳ���ֽ��� �Ƿ���� NV12�������ͼ���С 													// nRead == nFrameSize
		if (1)
		{

			// low8
			const clock_t begin_time1 = clock();
			const NvEncInputFrame* encoderInputFrame = enc.GetNextInputFrame();														// ����������ڻ�ȡ��һ�����õ����뻺������
			NvEncoderCuda::CopyToDeviceFrame(cuContext, pHostFrame.get(), 0, (CUdeviceptr)encoderInputFrame->inputPtr,				// ���������ݵ�CUDA�� pHostFrame.get()�����������ַ�����ơ�0x1f8cc20��
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
			NvEncoderCuda::CopyToDeviceFrame(cuContext, pHostFrame2.get(), 0, (CUdeviceptr)encoderInputFrame2->inputPtr,				// ���������ݵ�CUDA�� pHostFrame.get()�����������ַ�����ơ�0x1f8cc20��
				(int)encoderInputFrame2->pitch,
				enc2.GetEncodeWidth(),
				enc2.GetEncodeHeight(),
				CU_MEMORYTYPE_HOST,
				encoderInputFrame2->bufferFormat,
				encoderInputFrame2->chromaOffsets,
				encoderInputFrame2->numChromaPlanes);
			enc2.EncodeFrame(vPacket2);

			std::cout << "i = " << i << std::endl;
			//std::cout << "Encoding one frame = " << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
			/*Ӧ�ó���������EncodeFrame()������δѹ�������ݽ��б��룬
			��Щ�����Ѹ��Ƶ���GetNextInputFrame()������õ����뻺�����С�*/
			
		}
		else
		{
			enc.EndEncode(vPacket);		
			enc2.EndEncode(vPacket2);
																						
		}
		nFrame += (int)vPacket.size();

		t3.push_back(float(clock() - begin_time2) / CLOCKS_PER_SEC);
		const clock_t begin_time00 = clock();
		// ������뵽�˵ڼ�֡

		for (std::vector<uint8_t> &packet : vPacket)
		{
			// For each encoded packet
			fpOut.write(reinterpret_cast<char*>(packet.data()), packet.size());
		}

		for (std::vector<uint8_t> &packet2 : vPacket2)
		{
			// For each encoded packet
			fpOut2.write(reinterpret_cast<char*>(packet2.data()), packet2.size());
		}
	
		//t4.push_back(float(clock() - begin_time2) / CLOCKS_PER_SEC);
		i += 1;
		t6.push_back(float(clock() - begin_time00) / CLOCKS_PER_SEC);
		//if (name1 == "00816") break;
		// if (strcmp(end, name1)==0) break;
		if (i == encoder_num + 3) break;
	}
	std::cout << "Total time = " << float(clock() - begin) / CLOCKS_PER_SEC << std::endl;
	enc.DestroyEncoder();
	fpOut.close();


	std::cout << "Total frames encoded: " << nFrame << std::endl << "Saved in file " << szOutFilePath << std::endl;

	double sum_t1 = std::accumulate(std::begin(t1), std::end(t1), 0.0);
	double mean1 = sum_t1 / t1.size(); //��ֵ
	std::cout << "t1 ����ʱ��" << sum_t1 << std::endl;

	double sum_t_de = std::accumulate(std::begin(t2), std::end(t2), 0.0);
	double mean2 = sum_t_de / t2.size(); //��ֵ
	std::cout << "t2 �ҵĲ��� " << sum_t_de - sum_t1 << std::endl;

	double sum_t_wr = std::accumulate(std::begin(t3), std::end(t3), 0.0);
	double mean3 = sum_t_wr / t3.size(); //��ֵ
	std::cout << "t3 �����ʱ��" << sum_t_wr << std::endl;

	double sum_t2 = std::accumulate(std::begin(t4), std::end(t4), 0.0);
	double mean4 = sum_t2 / t4.size(); //��ֵ
	std::cout << "t4 " << sum_t2 << std::endl;

	double sum_t5 = std::accumulate(std::begin(t5), std::end(t5), 0.0);
	double mean5 = sum_t5 / t5.size(); //��ֵ
	std::cout << "t5 " << sum_t5 << std::endl;

	double sum_t6 = std::accumulate(std::begin(t6), std::end(t6), 0.0);
	double mean6 = sum_t6 / t5.size(); //��ֵ
	std::cout << "t5 " << sum_t6 << std::endl;
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

		if (!_stricmp(argv[i], "-num"))
		{
			++i;
			sscanf(argv[i], "%d", &encoder_num);
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




int main(int argc, char **argv) //�����˼����cmd ����
{
	char szInFilePath[256] = "C:\\Users\\USER\\Desktop\\PSNR\\Raw_tiff",
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

		ParseCommandLine(argc, argv, szInFilePath, nWidth, nHeight, encoder_num, eFormat, szOutFilePath, szOutFilePath2, encodeCLIOptions, iGpu); // ����������г�ʼ����

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

		EncodeCuda(cuContext, szInFilePath, nWidth, nHeight, encoder_num, eFormat, szOutFilePath, szOutFilePath2, &encodeCLIOptions, &encodeCLIOptions2);
	

}
	catch (const std::exception &ex)
	{
		std::cout << ex.what();
		return 1;
	}
	return 0;
}
