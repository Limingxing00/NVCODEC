# NVCODEC
I use NVENC and NVDEC to deal with 16bit medical image

## my computer
* windows10
* cuda 9.2
* opencv 3.0.0
* vs2013

## compiled appplication  
I have compiled encoder and decoder. You can directly use it. (.../NVCODEC/Comiled application)  
There are some samples:
`.\AppEncCuda.exe -h`  
You will watch help. The same as to AppDec.exe.
`.\AppDec.exe -h`

### encoder
`.\AppEncCuda.exe -i path_16bit -o ...\high.mkv -o2 ...\low.mkv -s 2048x788 -num 754 -if nv12 -gpu 0 -codec hevc -preset default -profile main -rc vbr_hq -fps 25 -gop 150 -qmin 1 -qmax 31  `  
"-i"   store original files in the directory (only 16bit TIFF, like 00001.tiff,00002.tiff...)  
"-o"   encoded high 8bits  
"-o2"   encoded low 8bits  
"-s"   image size  
"-num"  number to encode  
"-if"  color format(don't change it)  
"-gpu" choose a nvidia gpu(hevc needs more than GTX 950)  
"-codec" choose encoder type(hevc h264)  
"-qmax" I\P\B qmax  
### decoder
`.\AppDec.exe -i C:\Users\USER\Desktop\high.mkv -i2 C:\Users\USER\Desktop\low.mkv -o C:\Users\USER\Desktop\PSNR\nvdec\high -o2 C:\Users\USER\Desktop\PSNR\nvdec\low  -gpu 0`  
"-i"   encoded high 8bits  
"-i2"   encoded low 8bits  
"-o"  restored images
"-gpu" choose a nvidia gpu(hevc needs more than GTX 950)  

And you don't need care decoder parameters usually.  

## Source code(.../NVCODEC/CODE/)
I have little right to submit all codes. So you need to download [official sources](https://developer.nvidia.com/nvidia-video-codec-sdk#Download), if you have no account for NVIDIA. [Check it](https://download.csdn.net/download/qq_39575835/10890622).  
Please instead "...\Video_Codec_SDK_8.2.16\Samples\AppEncode\AppEncCuda\AppEncCuda.cpp" with mine. "...\Video_Codec_SDK_8.2.16\Samples\AppDecode\AppDec\AppDec.cpp" is same.  
By the way, don't forget OpenCV config in VS.
