#include "pch.h"
#include "OpenCVHelper.h"
#include "MemoryBuffer.h"
#include <vector>

using namespace OpenCVBridge;
using namespace Platform;
using namespace Windows::Graphics::Imaging;
using namespace Windows::Foundation;
using namespace Microsoft::WRL;
using namespace cv;
using namespace Windows::Foundation::Collections;
using namespace Platform::Collections;

//OpenCVHelper::OpenCVHelper(){}

bool OpenCVHelper::GetPointerToPixelData(SoftwareBitmap^ bitmap, unsigned char** pPixelData, unsigned int* capacity)
{
	BitmapBuffer^ bmpBuffer = bitmap->LockBuffer(BitmapBufferAccessMode::ReadWrite);
	IMemoryBufferReference^ reference = bmpBuffer->CreateReference();

	ComPtr<IMemoryBufferByteAccess> pBufferByteAccess;
	if ((reinterpret_cast<IInspectable*>(reference)->QueryInterface(IID_PPV_ARGS(&pBufferByteAccess))) != S_OK)
	{
		return false;
	}

	if (pBufferByteAccess->GetBuffer(pPixelData, capacity) != S_OK)
	{
		return false;
	}
	return true;
}

bool OpenCVHelper::TryConvert(SoftwareBitmap^ from, Mat& convertedMat)
{
	unsigned char* pPixels = nullptr;
	unsigned int capacity = 0;
	if (!GetPointerToPixelData(from, &pPixels, &capacity))
	{
		return false;
	}

	Mat mat(from->PixelHeight,
		from->PixelWidth,
		CV_8UC4, // assume input SoftwareBitmap is BGRA8
		(void*)pPixels);

	// shallow copy because we want convertedMat.data = pPixels
	// don't use .copyTo or .clone
	convertedMat = mat;
	return true;
}

void OpenCVHelper::Blur(SoftwareBitmap^ input, SoftwareBitmap^ output)
{
	Mat inputMat, outputMat;
	if (!(TryConvert(input, inputMat) && TryConvert(output, outputMat)))
	{
		return;
	}
	blur(inputMat, outputMat, cv::Size(15, 15));
}

 IVector<unsigned char>^ OpenCVHelper::TestChar() {
	 auto buff = ref new Vector<unsigned char>();
	 std::vector<unsigned char> ibuff;
	 std::vector<int> param = std::vector<int>(2);
	 param[0] = CV_IMWRITE_JPEG_QUALITY;
	 param[1] = 85;
	 Mat mt;
	 //imencode(".jpg", mt, ibuff, param);
	 
	 buff->Append(ibuff[0]);
	return buff;
}

 Array<unsigned char>^ OpenCVHelper::TestCharss() {
	 std::vector<unsigned char> ibuff{0,1,2};
	 
	 Array<unsigned char>^ buff = ref new Array<unsigned char>(&ibuff[0],3);
	 return buff;
 }

 Array<unsigned char>^ OpenCVHelper::GetCompressedBytes(SoftwareBitmap^ input) {
	 Mat inputMat;
	 Mat smallMat;
	 std::vector<unsigned char> ibuff;
	 if (!(TryConvert(input, inputMat))) {
		 Array<unsigned char>^ dummy = ref new Array<unsigned char>(0);
		 return dummy;
	 }
	 resize(inputMat, smallMat, cv::Size(), 0.25, 0.25);
	 std::vector<int> param = std::vector<int>(2);
	 param[0] = CV_IMWRITE_JPEG_QUALITY;
	 param[1] = 85;
	 //cvtColor(smallMat, smallMat, CV_BGRA2BGR);
	 imencode(".jpg", smallMat, ibuff, param);

	 Array<unsigned char>^ buff = ref new Array<unsigned char>(&ibuff[0], ibuff.size());

	 return buff;
 }

 int OpenCVHelper::TestArrayInput(int* testArr) {
	 int sum = 0;
	 return sum;
 }
