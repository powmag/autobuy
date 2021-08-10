#include<stdio.h>
#include<k4a/k4a.h>
#include<iostream>
#include<fstream>
#include<iostream>

using namespace std;

long WriteToFile(const char* fileName, void* buffer, size_t bufferSize)
{
	cout << bufferSize << endl;

	std::ofstream hFile;
	hFile.open(fileName, std::ios::out | std::ios::trunc | std::ios::binary);
	if (hFile.is_open())
	{
		hFile.write((char*)buffer, static_cast<std::streamsize>(bufferSize));
		hFile.close();
	}
	std::cout << "[Streaming Service] Color frame is stored in " << fileName << std::endl;

	return 0;
}

int main() {
	int device_count = k4a_device_get_installed_count();
	if (k4a_device_get_installed_count() == 0) {
		printf("디바이스가 감지되지 않았습니다.\n");
		return 1;
	}
	else
		printf("%d개의 디바이스가 검색되었습니다.\n", device_count);

	k4a_device_t device = NULL;
	for (int device_index = 0; device_index < device_count; device_index++) {
		if (k4a_device_open(device_index, &device) == 0) {
			printf("%d 번째 디바이스가 작동에 실패했습니다.\n", device_index);
			continue;
		}
	}

	//디바이스 환경 설정
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

	//디바이스 위 config 세팅에 맞춰 start
	if (K4A_FAILED(k4a_device_start_cameras(device, &config))) {
		printf("디바이스를 작동시키지 못했습니다.\n");
		k4a_device_close(device);
		return 1;
	}

	k4a_capture_t capture = NULL;
	const int32_t TIMEOUT_IN_MS = 1000;

	int frameCount = 1;
	while (frameCount-- > 0) {
		// Capture a depth frame
		switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
		{
		case K4A_WAIT_RESULT_SUCCEEDED:
			break;
		case K4A_WAIT_RESULT_TIMEOUT:
			printf("Timed out waiting for a capture\n");
			continue;
			break;
		case K4A_WAIT_RESULT_FAILED:
			printf("Failed to read a capture\n");
			return -1;
		}

		k4a_image_t image = k4a_capture_get_color_image(capture);
		uint8_t* img_buffer = k4a_image_get_buffer(image);
		size_t img_buffer_size = k4a_image_get_size(image);
		if (image)
		{
			printf(" | Color res:%4dx%4d stride:%5d ",
				k4a_image_get_height_pixels(image),
				k4a_image_get_width_pixels(image),
				k4a_image_get_stride_bytes(image));

			k4a_image_release(image);
		}
		else
		{
			printf(" | Color None                       ");
		}
		WriteToFile("color.jpg", img_buffer, img_buffer_size);
		k4a_capture_release(capture);
	}

	//디바이스 종료
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	return 0;
}
