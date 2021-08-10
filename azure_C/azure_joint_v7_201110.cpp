#define _CRT_SECURE_NO_WARNINGS
/***********************************************/
//				참고 사항					
// 실행 전 프로젝트 내 코드 있는 폴더에
// capture 폴더를 직접 만들어주세요
// 함수 찾기 귀찮아서 수동으로 만들었음 >_<
// 일정 시간 동안 사진 새로 안 찍히면 프로그램 끝납니다.
// v3 - 두 사람 찍혔을 때 제대로 동작
// v4 - 두 사람 이상일 때 제대로 돌아가는지 아직 모름
// v5 - TCP/IP 통신 클라이언트 연결
// v6 - crop 캡쳐 ok, but 원하는 좌표에서 불가능
/***********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#include <string.h>
#include <time.h>
#include <vector>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "write_file.h"
#include "tcp_ip_client.h"

using namespace std;
using namespace cv;

typedef struct _customer {
	int tracking_id;
	float pre_R_hand_joint_x = 1000;
	float pre_L_hand_joint_x = 1000;
	int price;
	k4abt_skeleton_t skeleton;
} Customer;

char* img_capture(k4a_capture_t cap, char* temp, char *directory, char* nn, char* ex, int* ft);
void crop_img(k4a_calibration_t *calibration, k4a_float3_t* source_point3d, char* org_cap_name, char* org_directory, char* new_directory);

int main() {
	int flag = 1;
	int frame_count = 0;			// frame_count 변수는 프레임 카운터용, 로그 보려고 남겨둔거
	int frame_take = 0;				// 처음 찍은거 확인용
	//int a = 0;						// rename 숫자 올리기에 쓸 거

	// 프로토콜 변수 (출발 지점, 목적 지점, 명령어, 프로토콜 전체 내용)
	const char* src_no = "2";
	const char* dest_no = "9";
	const char* inst = "buylist   ";
	char* protocol = (char*)malloc(sizeof(char) * 26);    // char 26개 크기(프로토콜 길이 + 프로토콜 전문)만큼 동적 메모리 할당
	
	// 키넥트에 찍히는 사람 넣는 visitor 변수
	// 시간 측정용 변수 2개
	// 키넥트 변수
	Customer visitor[10];
	clock_t now_tick, pre_tick;
	k4a_device_t device = NULL;

	// TCP_IP 통신 채널 오픈
/*
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata);		//윈속 초기화	

	SOCKET sock;
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);		//소켓 생성

	if (sock == -1) {
		return -1;
	}

	SOCKADDR_IN servaddr = { 0 };	//소켓 주소
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
	servaddr.sin_port = htons(PORT_NUM);

	int re = 0;
	re = connect(sock, (struct sockaddr*) & servaddr, sizeof(servaddr));	//연결 요청

	if (re == -1) {
		return -1;
	}

	_beginthread(RecvThreadPoint, 0, (void*)sock);
*/
	// TCP_IP 통신 채널 오픈 끝(OPEN END)

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

	pre_tick = clock();
	now_tick = clock();

	while (flag == 1) {
		char time_string[4];			// 문자가 된 정수 넣는 배열
		char dir[] = "./capture/";
		char org_dir[] = "./capture/";
		char new_dir[] = "./crop_cap2/";
		char new_name[] = "az_";
		char extention[] = ".jpg";
		char* cap_name;
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);

		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

			frame_count++;

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

				// 관절 좌표
				for (int i = 0; i < num_bodies; i++) {
					uint32_t id = k4abt_frame_get_body_id(body_frame, i);

					visitor[id - 1].tracking_id = id;

					k4abt_frame_get_body_skeleton(body_frame, i, &visitor[id - 1].skeleton);

					// z축 기준 설정 거리 이내로 처음 들어올 때 이미지 캡쳐
					if ((visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z <= 600 ||
						visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z <= 600) && frame_take == 0) {

						// 캡처
						cap_name = img_capture(sensor_capture, time_string, dir, new_name, extention, &frame_take);

						printf("%s\n", cap_name);

						// 최종 이용 시간 업데이트
						pre_tick = now_tick;

						printf("%d id picks up something!\n", id);
					}
					else if ((visitor[id - 1].pre_R_hand_joint_x <= 0 && visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x >= 0) ||
						(visitor[id - 1].pre_L_hand_joint_x <= 0 && visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.x >= 0)) {
						// 이전 프레임이랑 비교해서 손 빠져나갈 때 캡처

						// 캡처
						cap_name = img_capture(sensor_capture, time_string, dir, new_name, extention, &frame_take);
						
						printf("%s\n", cap_name);
						
						if (visitor[id - 1].pre_R_hand_joint_x <= 0) {
							crop_img(&sensor_calibration, &visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position, cap_name, org_dir, new_dir);
						}
						else {
							crop_img(&sensor_calibration, &visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position, cap_name, org_dir, new_dir);
						}
						
						// 최종 이용 시간 업데이트
						pre_tick = now_tick;

						// 프로토콜 보낼 문장 만들기
						strcpy(protocol, "0022");
						strcat(protocol, src_no);
						strcat(protocol, dest_no);
						strcat(protocol, inst);
				
						// 사물 분석 결과 프로토콜 추가 요망
						printf("%s %d\n", protocol, strlen(protocol));
						
						// 전문(Protocol) 보내기
						//send(sock, protocol, int(strlen(protocol)), 0);		//송신

						printf("%d id picks up something!\n", id);
					}

					printf("id : %d, joint_num : 15, joint_name : HAND_RIGHT\n", id);
					printf("오른손 x : %f, y : %f, z : %f\n", visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x,
						visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y, visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z);
					printf("왼손 x : %f, y : %f, z : %f\n", visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.x,
						visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.y, visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z);

					printf("왼손 이전 좌표 : %f, 오른손 이전 좌표 : %f\n", visitor[id - 1].pre_L_hand_joint_x, visitor[id - 1].pre_R_hand_joint_x);

					// 이전 프레임의 양손 z축 좌표 변경
					visitor[id - 1].pre_R_hand_joint_x = visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x;
					visitor[id - 1].pre_L_hand_joint_x = visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.x;
				}

				printf("%zu bodies are detected in %d frame!\n", num_bodies, frame_count);
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
		printf("현재 시간 : %d, 이전 시간 : %d\n\n", now_tick, pre_tick);

		if ((now_tick / CLOCKS_PER_SEC - pre_tick / CLOCKS_PER_SEC) > 15) {
			flag = 0;
		}
	}

	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	// 소켓 닫음
/*
	closesocket(sock);				//소켓 닫기    

	WSACleanup();					//윈속 해제화

	free(protocol);
*/
	// 소켓 닫는거 끝(CLOSE END)

	return 0;
}

char* img_capture(k4a_capture_t cap, char* temp, char* directory, char* nn, char* ex, int* ft) {
	int rename_result = 0;
	char base_name[] = "az.jpg";
	time_t tnow;
	struct tm* t;

	// 이미지 캡처 절차 시작
	k4a_image_t image = k4a_capture_get_color_image(cap);
	uint8_t* img_buffer = k4a_image_get_buffer(image);
	size_t img_buffer_size = k4a_image_get_size(image);

	if (image != NULL) {

		printf(" | Color16 res : %4d x %4d stride : %5d\n",
			k4a_image_get_height_pixels(image),
			k4a_image_get_width_pixels(image),
			k4a_image_get_stride_bytes(image));

		// 손 들어왔을때 캡처가 되긴 하는데 파일명을 바꿔서 계속 저장되게 해야돼
		WriteToFile(base_name, img_buffer, img_buffer_size);

		// 현재 시간 측정
		time(&tnow);
		t = (struct tm*) localtime(&tnow);

		// 이름 변경 전처리
		sprintf(temp, "%d", t->tm_year + 1900);
		strcat(nn, temp);
		sprintf(temp, "%02d", t->tm_mon + 1);
		strcat(nn, temp);
		sprintf(temp, "%02d", t->tm_mday);
		strcat(nn, temp);
		sprintf(temp, "%02d", t->tm_hour);
		strcat(nn, temp);
		sprintf(temp, "%02d", t->tm_min);
		strcat(nn, temp);
		sprintf(temp, "%02d", t->tm_sec);
		strcat(nn, temp);
		strcat(nn, ex);

		strcat(directory, nn);

		rename_result = rename(base_name, directory);		//이름변경

		if (rename_result == 0)
			printf("이름 변경 success\n");
		else if (rename_result == -1)
			printf("이름 변경 fail\n");

		// Release the image
		k4a_image_release(image);

		printf("찰칵 >_<\n");
		(*ft)++;

		return nn;
	}
	else {
		printf(" | Color16 None\n");

		return 0;
	}
}

void crop_img(k4a_calibration_t* calibration, k4a_float3_t* source_point3d, char* org_cap_name, char* org_directory, char* new_directory) {
	//int valid = 0;
	
	strcat(org_directory, org_cap_name);
/*
	// k4a project function
	vector<k4a_float2_t> k4a_points_2d(1);

	k4a_calibration_3d_to_2d(calibration,
		source_point3d,
		K4A_CALIBRATION_TYPE_COLOR,
		K4A_CALIBRATION_TYPE_DEPTH,
		&k4a_points_2d[0],
		&valid);
*/	
	Mat img = imread(org_directory);

	// 관심 영역 설정(set ROI(X, Y, W, H))
	Rect rect(source_point3d->xyz.x + 1700, source_point3d->xyz.y + 1400, 500, 500);

	// 관심 영역 자르기(Crop ROI)
	Mat subImg = img(rect);

	//imshow("hermione", subImg);
	//waitKey(3000);
	strcat(new_directory, org_cap_name);

	// 이미지 저장하기
	imwrite(new_directory, subImg);

	printf("crop 찰칵 >_<\n");
}
