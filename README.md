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
You will watch help.  

`.\AppEncCuda.exe -i path_16bit -o ...\high.mkv -o2 ...\low.mkv -s 2048x788 -num 754 -if nv12 -gpu 0 -codec hevc -preset default -profile main -rc vbr_hq -fps 25 -gop 150 -qmin 1 -qmax 31  `  
"-i"   store original files in the directory (only 16bit TIFF, like 00001.tiff,00002.tiff...)  
"-o"   encoded high 8bits
"-o2"   encoded low 8bits  
"-s"   image size  
"-num"  number to encode  
"-if"  color format(don't change)  
"-gpu" choose a nvidia gpu(hevc needs more than GTX 950)  
"-codec" choose encoder type(hevc h264)  
"-qmax" I\P\B qmax
