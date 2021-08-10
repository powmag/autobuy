#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#include <string.h>
#include <time.h>

#include <k4a/k4a.h>
#include <k4abt.h>
#include "write_file.h"

int main() {
	int flag = 1;\
	clock_t now_tick, pre_tick;
	k4a_device_t device = NULL;
	
	// 디바이스 열기
	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
		printf("Failed to open device\n");
		return -1;
	}

	// Start camera. Make sure depth camera is enabled.
	// 디바이스 구성 구조체 변수
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_3072P;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;

	// 디바이스 시작
	if (k4a_device_start_cameras(device, &deviceConfig) != K4A_RESULT_SUCCEEDED) {
		printf("Failed to start device\n");
		exit(1);
	}

	// 센서 보정 구조체 변수
	k4a_calibration_t sensor_calibration;

	if (k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration)
		!= K4A_RESULT_SUCCEEDED) {
		printf("Failed to get depth camera calibration\n");
		exit(1);
	}
	printf("해상도 : %d\n", deviceConfig.color_resolution);

	// 신체 추적기 만들기
	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

	if (k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker) != K4A_RESULT_SUCCEEDED) {
		printf("Failed to create tracker\n");
		exit(1);
	}

	// frame_count 변수는 프레임 카운터용, 로그 보려고 남겨둔거
	int frame_count = 0;
	int frame_take = 0;				// 처음 찍은거 확인용
	int a = 0;						// rename 숫자 올리기에 쓸 거
	float pre_R_hand_joint_z = 1000;
	float pre_L_hand_joint_z = 1000;

	pre_tick = clock();
	now_tick = clock();

	while(flag == 1) {
		int rename_result = 0;
		char s[5];			// 문자가 된 정수 넣는 배열
		char new_name[] = "./capture/capture";
		char extention[] = ".jpg";
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);

		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
			frame_count++;
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

			if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!\n");
				break;
			}
			else if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
				printf("Error! Add capture to tracker process queue failed!\n");
				break;
			}

			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);

			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
				// Successfully popped the body tracking result. Start your processing

				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				k4abt_skeleton_t skeleton;

				// 관절 좌표
				for (int i = 0; i < num_bodies; i++) {
					char base_name[] = "./capture/capture.jpg";
					uint32_t id = k4abt_frame_get_body_id(body_frame, i);

					k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

					// z축 기준 설정 거리 이내로 처음 들어올 때 이미지 캡쳐
					if ((skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z <= 600 ||
						skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z <= 600) && frame_take == 0) {

						// 이미지 캡처 절차 시작
						k4a_image_t image = k4a_capture_get_color_image(sensor_capture);
						uint8_t *img_buffer = k4a_image_get_buffer(image);
						size_t img_buffer_size = k4a_image_get_size(image);

						if (image != NULL) {

							printf(" | Color16 res : %4d x %4d stride : %5d\n",
								k4a_image_get_height_pixels(image),
								k4a_image_get_width_pixels(image),
								k4a_image_get_stride_bytes(image));

							// 손 들어왔을때 캡처가 되긴 하는데 파일명을 바꿔서 계속 저장되게 해야돼
							WriteToFile(base_name, img_buffer, img_buffer_size);
							
							// 이름 변경 전처리
							sprintf(s, "%d", a++);
							strcat(new_name, s);
							strcat(new_name, extention);
							printf("%s\n", new_name);

							rename_result = rename(base_name, new_name);		//이름변경

							if (rename_result == 0)
								printf("이름 변경 success\n");
							else if (rename_result == -1)
								printf("이름 변경 fail\n");

							// Release the image
							k4a_image_release(image);

							printf("찰칵1 >_<\n");
							frame_take++;
						}
						else {
							printf(" | Color16 None\n");
						}

						// 최종 이용 시간 업데이트
						pre_tick = now_tick;
					
						printf("%d id picks up something!\n", id);
						break;
					}
					else if ((pre_R_hand_joint_z <= 600 && skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z >= 600) ||
								(pre_L_hand_joint_z <= 600 && skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z >= 600)) {
					// 이전 프레임이랑 비교해서 손 빠져나갈 때 캡처

						// 이미지 캡처 절차 시작
						k4a_image_t image = k4a_capture_get_color_image(sensor_capture);
						uint8_t* img_buffer = k4a_image_get_buffer(image);
						size_t img_buffer_size = k4a_image_get_size(image);

						if (image != NULL) {
							printf(" | Color16 res : %4d x %4d stride : %5d\n",
								k4a_image_get_height_pixels(image),
								k4a_image_get_width_pixels(image),
								k4a_image_get_stride_bytes(image));

							// 손 들어왔을때 캡처가 되긴 하는데 파일명을 바꿔서 계속 저장되게 해야돼
							WriteToFile(base_name, img_buffer, img_buffer_size);

							// 이름 변경 전처리
							sprintf(s, "%d", a++);
							strcat(new_name, s);
							strcat(new_name, extention);
							printf("%s\n", new_name);

							rename_result = rename(base_name, new_name);		//이름변경

							if (rename_result == 0)
								printf("이름 변경 success\n");
							else if (rename_result == -1)
								printf("이름 변경 fail\n");

							// Release the image
							k4a_image_release(image);

							printf("찰칵2 >_<\n");
						}
						else {
							printf(" | Color16 None\n");
						}

						// 최종 이용 시간 업데이트
						pre_tick = now_tick;

						printf("%d id picks up something!\n", id);
						break;
					}

					printf("id : %d, joint_num : 15, joint_name : HAND_RIGHT\n", id);
					printf("오른손 x : %f, y : %f, z : %f\n", skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x,
						skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y, skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z);
				}

				// 이전 프레임의 양손 z축 좌표 but 한 사람만 가능
				// 여러 사람으로 하려면 수정 필요
				pre_R_hand_joint_z = skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z;
				pre_L_hand_joint_z = skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z;

				printf("왼손 이전 좌표 : %f, 오른손 이전 좌표 : %f\n", pre_L_hand_joint_z, pre_R_hand_joint_z);

				printf("%zu bodies are detected in %d frame!\n\n", num_bodies, frame_count);
				k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!\n");
				break;
			}
			else {
				printf("Pop body frame result failed!\n");
				break;
			}

			k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
			fflush(stdout);
		}
		else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Error! Get depth frame time out!\n");
			break;
		}
		else {
			printf("Get depth capture returned error: %d\n", get_capture_result);
			break;
		}

		now_tick = clock();
		printf("현재 시간 : %d, 이전 시간 : %d\n", now_tick, pre_tick);
		
		if ((now_tick / CLOCKS_PER_SEC - pre_tick / CLOCKS_PER_SEC) > 5)
			flag = 0;
	}

	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	return 0;
}
