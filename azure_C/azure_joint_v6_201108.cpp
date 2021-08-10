#define _CRT_SECURE_NO_WARNINGS
/***********************************************/
//				���� ����					
// ���� �� ������Ʈ �� �ڵ� �ִ� ������
// capture ������ ���� ������ּ���
// �Լ� ã�� �����Ƽ� �������� ������� >_<
// ���� �ð� ���� ���� ���� �� ������ ���α׷� �����ϴ�.
// v3 - �� ��� ������ �� ����� ����
// v4 - �� ��� �̻��� �� ����� ���ư����� ���� ��
// v5 - TCP/IP ��� Ŭ���̾�Ʈ ����
// v6 - crop ĸ�� ok, but ���ϴ� ��ǥ���� �Ұ���
/***********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#include <string.h>
#include <time.h>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "write_file.h"
#include "tcp_ip_client.h"

using namespace std;
using namespace cv;

typedef struct _customer {
	int tracking_id;
	float pre_R_hand_joint_z = 1000;
	float pre_L_hand_joint_z = 1000;
	int price;
	k4abt_skeleton_t skeleton;
} Customer;

char* img_capture(k4a_capture_t cap, char* temp, char *directory, char* nn, char* ex, int* ft);
void crop_img(float hand_x, float hand_y, char* org_cap_name, char* org_directory, char* new_directory);

int main() {
	int flag = 1;
	int frame_count = 0;			// frame_count ������ ������ ī���Ϳ�, �α� ������ ���ܵа�
	int frame_take = 0;				// ó�� ������ Ȯ�ο�
	//int a = 0;						// rename ���� �ø��⿡ �� ��

	// �������� ���� (��� ����, ���� ����, ��ɾ�, �������� ��ü ����)
	const char* src_no = "2";
	const char* dest_no = "9";
	const char* inst = "buylist   ";
	char* protocol = (char*)malloc(sizeof(char) * 26);    // char 26�� ũ��(�������� ���� + �������� ����)��ŭ ���� �޸� �Ҵ�
	
	// Ű��Ʈ�� ������ ��� �ִ� visitor ����
	// �ð� ������ ���� 2��
	// Ű��Ʈ ����
	Customer visitor[10];
	clock_t now_tick, pre_tick;
	k4a_device_t device = NULL;

	// TCP_IP ��� ä�� ����
/*
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata);		//���� �ʱ�ȭ	

	SOCKET sock;
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);		//���� ����

	if (sock == -1) {
		return -1;
	}

	SOCKADDR_IN servaddr = { 0 };	//���� �ּ�
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
	servaddr.sin_port = htons(PORT_NUM);

	int re = 0;
	re = connect(sock, (struct sockaddr*) & servaddr, sizeof(servaddr));	//���� ��û

	if (re == -1) {
		return -1;
	}

	_beginthread(RecvThreadPoint, 0, (void*)sock);
*/
	// TCP_IP ��� ä�� ���� ��(OPEN END)

	// ����̽� ����
	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
		printf("Failed to open device\n");
		return -1;
	}

	// Start camera. Make sure depth camera is enabled.
	// ����̽� ���� ����ü ����
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_3072P;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;

	// ����̽� ����
	if (k4a_device_start_cameras(device, &deviceConfig) != K4A_RESULT_SUCCEEDED) {
		printf("Failed to start device\n");
		exit(1);
	}

	// ���� ���� ����ü ����
	k4a_calibration_t sensor_calibration;

	if (k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration)
		!= K4A_RESULT_SUCCEEDED) {
		printf("Failed to get depth camera calibration\n");
		exit(1);
	}
	printf("�ػ� : %d\n", deviceConfig.color_resolution);

	// ��ü ������ �����
	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

	if (k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker) != K4A_RESULT_SUCCEEDED) {
		printf("Failed to create tracker\n");
		exit(1);
	}

	pre_tick = clock();
	now_tick = clock();

	while (flag == 1) {
		char time_string[4];			// ���ڰ� �� ���� �ִ� �迭
		char dir[] = "./capture/";
		char org_dir[] = "./capture/";
		char new_dir[] = "./crop_cap/";
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

				// ���� ��ǥ
				for (int i = 0; i < num_bodies; i++) {
					uint32_t id = k4abt_frame_get_body_id(body_frame, i);

					visitor[id - 1].tracking_id = id;

					k4abt_frame_get_body_skeleton(body_frame, i, &visitor[id - 1].skeleton);

					// z�� ���� ���� �Ÿ� �̳��� ó�� ���� �� �̹��� ĸ��
					if ((visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z <= 600 ||
						visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z <= 600) && frame_take == 0) {

						// ĸó
						cap_name = img_capture(sensor_capture, time_string, dir, new_name, extention, &frame_take);

						printf("%s\n", cap_name);

						// ���� �̿� �ð� ������Ʈ
						pre_tick = now_tick;

						printf("%d id picks up something!\n", id);
					}
					else if ((visitor[id - 1].pre_R_hand_joint_z <= 600 && visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z >= 600) ||
						(visitor[id - 1].pre_L_hand_joint_z <= 600 && visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z >= 600)) {
						// ���� �������̶� ���ؼ� �� �������� �� ĸó

						// ĸó
						cap_name = img_capture(sensor_capture, time_string, dir, new_name, extention, &frame_take);
						
						printf("%s\n", cap_name);
						
						if (visitor[id - 1].pre_R_hand_joint_z <= 600) {
							crop_img(visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x, 
								visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y, cap_name,
								org_dir, new_dir);
						}
						else {
							crop_img(visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.x, 
								visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.y, cap_name,
								org_dir, new_dir);
						}
						
						// ���� �̿� �ð� ������Ʈ
						pre_tick = now_tick;

						// �������� ���� ���� �����
						strcpy(protocol, "0022");
						strcat(protocol, src_no);
						strcat(protocol, dest_no);
						strcat(protocol, inst);
				
						// �繰 �м� ��� �������� �߰� ���
						printf("%s %d\n", protocol, strlen(protocol));
						
						// ����(Protocol) ������
						//send(sock, protocol, int(strlen(protocol)), 0);		//�۽�

						printf("%d id picks up something!\n", id);
					}

					printf("id : %d, joint_num : 15, joint_name : HAND_RIGHT\n", id);
					printf("������ x : %f, y : %f, z : %f\n", visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x,
						visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y, visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z);

					printf("�޼� ���� ��ǥ : %f, ������ ���� ��ǥ : %f\n", visitor[id - 1].pre_L_hand_joint_z, visitor[id - 1].pre_R_hand_joint_z);

					// ���� �������� ��� z�� ��ǥ ����
					visitor[id - 1].pre_R_hand_joint_z = visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z;
					visitor[id - 1].pre_L_hand_joint_z = visitor[id - 1].skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.z;
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
		printf("���� �ð� : %d, ���� �ð� : %d\n\n", now_tick, pre_tick);

		if ((now_tick / CLOCKS_PER_SEC - pre_tick / CLOCKS_PER_SEC) > 15) {
			flag = 0;
		}
	}

	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	// ���� ����
/*
	closesocket(sock);				//���� �ݱ�    

	WSACleanup();					//���� ����ȭ

	free(protocol);
*/
	// ���� �ݴ°� ��(CLOSE END)

	return 0;
}

char* img_capture(k4a_capture_t cap, char* temp, char* directory, char* nn, char* ex, int* ft) {
	int rename_result = 0;
	char base_name[] = "az.jpg";
	time_t tnow;
	struct tm* t;

	// �̹��� ĸó ���� ����
	k4a_image_t image = k4a_capture_get_color_image(cap);
	uint8_t* img_buffer = k4a_image_get_buffer(image);
	size_t img_buffer_size = k4a_image_get_size(image);

	if (image != NULL) {

		printf(" | Color16 res : %4d x %4d stride : %5d\n",
			k4a_image_get_height_pixels(image),
			k4a_image_get_width_pixels(image),
			k4a_image_get_stride_bytes(image));

		// �� �������� ĸó�� �Ǳ� �ϴµ� ���ϸ��� �ٲ㼭 ��� ����ǰ� �ؾߵ�
		WriteToFile(base_name, img_buffer, img_buffer_size);

		// ���� �ð� ����
		time(&tnow);
		t = (struct tm*) localtime(&tnow);

		// �̸� ���� ��ó��
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

		rename_result = rename(base_name, directory);		//�̸�����

		if (rename_result == 0)
			printf("�̸� ���� success\n");
		else if (rename_result == -1)
			printf("�̸� ���� fail\n");

		// Release the image
		k4a_image_release(image);

		printf("��Ĭ >_<\n");
		(*ft)++;

		return nn;
	}
	else {
		printf(" | Color16 None\n");

		return 0;
	}
}

void crop_img(float hand_x, float hand_y, char* org_cap_name, char* org_directory, char* new_directory) {
	strcat(org_directory, org_cap_name);
	printf("%s\n", org_directory);
	Mat img = imread(org_directory);

	// ���� ���� ����(set ROI(X, Y, W, H))
	Rect rect(1000, 1000, 500, 500);

	// ���� ���� �ڸ���(Crop ROI)
	Mat subImg = img(rect);

	//imshow("hermione", subImg);
	//waitKey(3000);
	strcat(new_directory, org_cap_name);

	// �̹��� �����ϱ�
	imwrite(new_directory, subImg);

	printf("crop ��Ĭ >_<\n");
}
