//���Ժ�������ͷ���ɹ���⵽����
#include <cstdlib>
#include <cstring>
#include <iostream>
#include "Windows.h"
#include "HCNetSDK.h"
#include "plaympeg4.h"
#include <opencv2\opencv.hpp>
#include <time.h>
#include <stdio.h>
//#include <opencv2/opencv.hpp>
#include "facedetect-dll.h"
#pragma comment(lib,"libfacedetect.lib")
//#pragma comment(lib,"libfacedetect-x64.lib")

//define the buffer size. Do not change the size!
#define DETECT_BUFFER_SIZE 0x20000

using namespace std;
using namespace cv;

LONG nPort = -1;
LONG lRealPlayHandle;
int goto_preset = 0;
int flag = 0;
Mat result_frontal_surveillance;
NET_DVR_POINT_FRAME  posdata;
int x;
int y;
int w;
int h;
Size size = Size(1280, 720);
VideoWriter writer;

volatile int gbHandling = 3;

//����ص� ��ƵΪYUV����(YV12)����ƵΪPCM����
void CALLBACK DecCBFun(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long nReserved2)
{
	if (gbHandling)
	{
		gbHandling--;
		return;
	}

	long lFrameType = pFrameInfo->nType;
	if (lFrameType == T_YV12)
	{

		Mat pImg(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
		Mat src(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, pBuf);
		cvtColor(src, pImg, CV_YUV2BGR_YV12);
		//cvtColor(pImg, pImg, COLOR_BGR2GRAY);
		//printf("successaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!");
		//  Sleep(-1);

		clock_t start_time = clock();
		cv::Mat gray;
		cv::cvtColor(pImg, gray, CV_BGR2GRAY);
		int * pResults = NULL;
		//pBuffer is used in the detection functions.
		//If you call functions in multiple threads, please create one buffer for each thread!
		unsigned char * pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
		if (!pBuffer)
		{
			fprintf(stderr, "Can not alloc buffer.\n");
			//return -1;
		}

		int doLandmark = 1;
		if (!flag)
		{
			pResults = facedetect_frontal_surveillance(pBuffer, (unsigned char*)(gray.ptr(0)), gray.cols, gray.rows, (int)gray.step,
				1.2f, 2, 48, 0, doLandmark);
			//result_frontal_surveillance = pImg.clone();
			//print the detection results
			for (int i = 0; i < (pResults ? *pResults : 0); i++)
			{
				short * p = ((short*)(pResults + 1)) + 142 * i;
				//�������ο�Խ����ϵ�������
				x = p[0];
				y = p[1];
				w = p[2];
				h = p[3];
				int neighbors = p[4];
				int angle = p[5];


				printf("face_rect=[%d, %d, %d, %d], neighbors=%d, angle=%d\n", x, y, w, h, neighbors, angle);
				rectangle(pImg, Rect(x, y, w, h), Scalar(0, 255, 0), 2);
				if (doLandmark)
				{
					for (int j = 0; j < 68; j++)
						circle(pImg, Point((int)p[6 + 2 * j], (int)p[6 + 2 * j + 1]), 1, Scalar(0, 255, 0));
				}
				posdata.xTop = (int)(x * 255 / 1280);
				posdata.xBottom = (int)((x + w) * 255 / 1280);// (int)((target_tmp.x + target_tmp.width) / std_cols * 255);
				posdata.yTop = (int)(y * 255 / 720);
				posdata.yBottom = (int)((y + h) * 255 / 720);// (int)((target_tmp.y + target_tmp.height) / std_rows * 255);
				posdata.bCounter = 1;

				if (!NET_DVR_PTZSelZoomIn_EX(0, 1, &posdata))
				{
					auto a = NET_DVR_GetLastError();
					printf("Zoom error!!");
				}

				else
				{
					printf("Zoom success!!");
					flag = 1;

				}
			}
			

		}		

		
	
		//printf("%d faces detected.\n", (pResults ? *pResults : 0));
		
		imshow("IPCamera", pImg);
		
		
		writer.write(pImg);
		goto_preset++;
		if (goto_preset == 10)
		{

			////////////goto_preset//////////////////
			if (!NET_DVR_PTZPreset(lRealPlayHandle, GOTO_PRESET, 1))
			{
				printf("goto preset error");
			}
			else
			{
				printf("goto preset success");
				goto_preset = 0;
				flag = 0;
			}
		}
		
		clock_t end_time = clock();
		clock_t totaltime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		cout << "\n�˳��������ʱ��Ϊ" << totaltime << "�룡" << endl;
		
		
		
	
		

		waitKey(1);

		//release the buffer
		free(pBuffer);

	}

	gbHandling = 3;

}


///ʵʱ���ص�
void CALLBACK fRealDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
{

	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD: //ϵͳͷ

		if (!PlayM4_GetPort(&nPort))  //��ȡ���ſ�δʹ�õ�ͨ����
		{
			break;
		}
		//m_iPort = lPort; //��һ�λص�����ϵͳͷ������ȡ�Ĳ��ſ�port�Ÿ�ֵ��ȫ��port���´λص�����ʱ��ʹ�ô�port�Ų���
		if (dwBufSize > 0)
		{
			if (!PlayM4_SetStreamOpenMode(nPort, STREAME_REALTIME))  //����ʵʱ������ģʽ
			{
				break;
			}

			if (!PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 10 * 1024 * 1024)) //�����ӿ�
			{
				break;
			}

			if (!PlayM4_Play(nPort, NULL)) //���ſ�ʼ
			{
				break;
			}
			if (!PlayM4_SetDecCallBack(nPort, DecCBFun))
			{
				break;
			}
		}
		break;
	case NET_DVR_STREAMDATA:   //��������
		if (dwBufSize > 0 && nPort != -1)
		{
			if (!PlayM4_InputData(nPort, pBuffer, dwBufSize))
			{
				cout << "error" << PlayM4_GetLastError(nPort) << endl;
				break;
			}
		}
		break;
	default: //��������
		if (dwBufSize > 0 && nPort != -1)
		{
			if (!PlayM4_InputData(nPort, pBuffer, dwBufSize))
			{
				break;
			}
		}
		break;
	}
}


void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
	char tempbuf[256] = { 0 };
	switch (dwType)
	{
	case EXCEPTION_RECONNECT:    //Ԥ��ʱ����
		printf("----------reconnect--------%d\n", time(NULL));
		break;
	default:
		break;
	}
}
void PTZControlAll(LONG lRealHandle, DWORD dwPTZCommand, DWORD dwStop, int Speed)
{
	if (lRealHandle >= 0)
	{
		BOOL ret;
		if (Speed >= 1)
		{
			ret = NET_DVR_PTZControlWithSpeed(lRealHandle, dwPTZCommand, dwStop, Speed);
			if (!ret)
			{
				//MessageBox("��̨����ʧ��!");
				printf("-------��̨����ʧ��!-----------%d\n");
				DWORD a = NET_DVR_GetLastError();
				
				return;
			}
		}
		else
		{
			ret = NET_DVR_PTZControl(lRealHandle, dwPTZCommand, dwStop);
			if (!ret)
			{
				//MessageBox("��̨����ʧ��!");
				printf("-------��̨����ʧ��!-----------%d\n");
				return;
			}
		}
	}

}

void main()
{

	//---------------------------------------
	// ��ʼ��
	NET_DVR_Init();
	//��������ʱ��������ʱ��
	NET_DVR_SetConnectTime(2000, 1);
	NET_DVR_SetReconnect(10000, true);


	//---------------------------------------
	// ע���豸
	LONG lUserID;
	NET_DVR_DEVICEINFO_V30 struDeviceInfo;
	lUserID = NET_DVR_Login_V30("192.168.1.186", 8000, "admin", "12345", &struDeviceInfo);
	if (lUserID < 0)
	{
		printf("Login error, %d\n", NET_DVR_GetLastError());
		NET_DVR_Cleanup();
		return;
	}

	//---------------------------------------
	//�����쳣��Ϣ�ص�����
	NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);

	//---------------------------------------
	//����Ԥ�������ûص�������
	//LONG lRealPlayHandle;
	cvNamedWindow("Mywindow", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("IPCamera", CV_WINDOW_AUTOSIZE);
	writer.open("a2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, size, true);

	HWND  h = (HWND)cvGetWindowHandle("Mywindow");
	if (h == 0)
	{
		cout << "���ڴ���ʧ��" << endl;
	}


	NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
	struPlayInfo.hPlayWnd = h;         //��ҪSDK����ʱ�����Ϊ��Чֵ����ȡ��������ʱ����Ϊ��
	struPlayInfo.lChannel = 1;           //Ԥ��ͨ����
	struPlayInfo.dwStreamType = 0;       //0-��������1-��������2-����3��3-����4���Դ�����
	struPlayInfo.dwLinkMode = 0;         //0- TCP��ʽ��1- UDP��ʽ��2- �ಥ��ʽ��3- RTP��ʽ��4-RTP/RTSP��5-RSTP/HTTP

	lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, fRealDataCallBack, NULL);

	if (lRealPlayHandle < 0)
	{
		printf("NET_DVR_RealPlay_V40 error\n");
		printf("2222222222 %d\n", NET_DVR_GetLastError());
		NET_DVR_Logout(lUserID);
		NET_DVR_Cleanup();
		return;
	}

	if(!NET_DVR_PTZPreset(lRealPlayHandle,SET_PRESET,1))
	{
		printf("set preset error");
	}
	else
	{
		printf("set preset success");
	}
	
	//��̨����
	//PTZControlAll(lRealPlayHandle, PAN_LEFT, 0, 3);
	//Sleep(1000);
	//PTZControlAll(lRealPlayHandle, PAN_LEFT, 1, 3);
	
	//PTZControlAll(lRealPlayHandle, PAN_RIGHT, 0, 3);
	//Sleep(3000);
	//PTZControlAll(lRealPlayHandle, PAN_RIGHT, 1, 3);

	//PTZControlAll(lRealPlayHandle, TILT_UP, 0, 5);
	//Sleep(500);
	//PTZControlAll(lRealPlayHandle, TILT_UP, 1, 5);

	//PTZControlAll(lRealPlayHandle, TILT_DOWN, 0, 5);
	//Sleep(500);
	//PTZControlAll(lRealPlayHandle, TILT_DOWN, 1, 5);

	


	waitKey();

	Sleep(-1);
	//---------------------------------------
	//�ر�Ԥ��
	NET_DVR_StopRealPlay(lRealPlayHandle);
	//ע���û�
	NET_DVR_Logout(lUserID);
	//�ͷ�SDK��Դ
	NET_DVR_Cleanup();

	return;
}
