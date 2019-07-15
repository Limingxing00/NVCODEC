# NVCODEC
I use NVENC and NVDEC to deal with 16bit medical image
![pipeline](https://github.com/Limingxing00/NVCODEC/blob/master/CODE/img.jpg)

### Tip. My project is different from [NVIDIA CODEC](https://developer.nvidia.com/designworks/video_codec_sdk/downloads/v9.0). I modified the source code to make it suitable for grayscale images (only for grayscale images).  
Specifically, we modified ".../Samples/AppEncode/AppEncCuda/AppEncCuda.cpp" to control the encoder, and modified ".../Samples/AppDecode/AppDec/AppDec.cpp" to control the decoder. Different modifications are shown [here].(https://github.com/Limingxing00/NVCODEC/tree/master/CODE)

## my computer
* windows10
* cuda 9.2
* opencv 3.0.0
* vs2015

## compiled appplication(There are some troubles)  
.../NVCODEC/Comiled application  
There are some samples:
`.\AppEncCuda.exe -h`  
You will watch help. The same as to AppDec.exe.
`.\AppDec.exe -h`

### encoder
`.\AppEncCuda.exe -i path_16bit -o ...\high.mkv -o2 ...\low.mkv -s 2048x788 -num 754 -if nv12 -gpu 0 -codec hevc -preset default -profile main -rc vbr_hq -fps 25 -gop 150 -qmin 1 -qmax 31  `  
"-i"   store original files in the directory (5-dimensional dataset encapsulated in OME-TIFF, such as Raw.ome.tif)
"-o"   encoded high 6/8/10bits  
"-o2"   encoded low 6/8/10bits  
"-s"   image size(default: 2048x788) 
"-if"  color format(don't change it)  
"-gpu" choose a nvidia gpu(hevc needs more than GTX 950)  
"-codec" choose encoder type(hevc h264)  
"-qmax" I\P\B qmax  
### decoder
`.\AppDec.exe -i ...\high.mkv -i2 ...\low.mkv -o2 path_16bit  -gpu 0`  
"-i"   encoded high 6/8/10bits  
"-i2"   encoded low 6/8/10bits  
"-o2"  restored images
"-gpu" choose a nvidia gpu(hevc needs more than GTX 950)  

And you don't need care decoder parameters usually.  

## Source code(.../NVCODEC/CODE/)
I have little right to submit all codes. So you need to download [official sources](https://developer.nvidia.com/nvidia-video-codec-sdk#Download), if you have no account for NVIDIA. [Check it](https://download.csdn.net/download/qq_39575835/10890622).  
Please instead "...\Video_Codec_SDK_8.2.16\Samples\AppEncode\AppEncCuda\AppEncCuda.cpp" with mine. "...\Video_Codec_SDK_8.2.16\Samples\AppDecode\AppDec\AppDec.cpp" is same.  
By the way, don't forget OpenCV config in VS.

