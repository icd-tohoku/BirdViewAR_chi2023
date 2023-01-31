#pragma once
#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <collection.h>

namespace OpenCVBridge
{
	public ref class OpenCVHelper sealed
	{
	public:
		OpenCVHelper() {}

		void Blur(
			Windows::Graphics::Imaging::SoftwareBitmap^ input,
			Windows::Graphics::Imaging::SoftwareBitmap^ output);

		Windows::Foundation::Collections::IVector<unsigned char>^ TestChar();
		Platform::Array<unsigned char>^ TestCharss();
		Platform::Array<unsigned char>^ GetCompressedBytes(Windows::Graphics::Imaging::SoftwareBitmap^ input);
		int TestArrayInput(int* testArr);

	private:
		// helper functions for getting a cv::Mat from SoftwareBitmap
		bool TryConvert(Windows::Graphics::Imaging::SoftwareBitmap^ from, cv::Mat& convertedMat);
		bool GetPointerToPixelData(Windows::Graphics::Imaging::SoftwareBitmap^ bitmap,
			unsigned char** pPixelData, unsigned int* capacity);
	};
}

