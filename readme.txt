The zip file contains custom recording software and custom codes including output files to record and extract the neural signal data using a developed wireless system. 

1. System requirements
  - Operating system: Windows 10
  - Required software: IAR Embedded workbench, Tera Term, Visual studio 2019 or Visual studio SDK, Matlab 2016a

2. Installation guide
  - The software runs after installing Visual studio 2019 or Visual studio SDK.
  - The code runs inside Matlab and doesn't require any installation. 

3. Writing the custom code into the wireless communication module
  - Using the IDE software such as IAR Embedded workbench, compile and run the custom code (SPPLEDemo.c).
  - Using the USB MSP430 downloader, write the code into the microcontroller (MSP430) on the wireless communication module.
  - If it succeeds without errors, you are ready to measure the neural signals using wireless communication module.

4. Data acquisition
  - Using the Tera Term open-source software, set up the Bluetooth receiver connected to the computer through the macro file(teraterm.ttl) and connect it to the wireless system (i.e., transmitter). (See screenshot 1)
  - Run the custom recording software.
  - An outfile in .txt format is created automatically, and the neural signal is saved. (outfile.txt contains information such as time, channel, and amplitude.)
  - Also, we can simultaneously observe the neural signals measured for each channel through the custom recording software. (see screenshot 2)

5. Data extraction
  - Run the Matlab and open the" data_extraction.m" file. 
  - Move to the location of the input file(outfile.txt).
  - Run (F5)
- Expected output: recorded dataset in .mat format (chX is raw data and f_chX is noise-filtered data by Fourier transform.)
- Expected run time: about 1 minute in case of data recorded for 5 minutes (depend on data size and computer performance)

6. Neural signal analysis
- The noise-filtered data are analyzed using principal component analysis (PCA) and the k-means clustering algorithm based on the python (https://github.com/akcarsten/spike_sorting) to detect the neural spikes in recorded data.

