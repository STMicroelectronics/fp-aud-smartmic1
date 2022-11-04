/**
  ******************************************************************************
  * @file    
  * @author  SRA
  * @brief 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
#include <iostream>
#include <fstream>

#ifndef CCOMPORT_NO_QT
#include <QThread>
#include <QCoreApplication>
#endif

#ifdef _WIN32
#include <conio.h>
#else
extern "C" {
#include "kbhit.h"
}
#endif
#include <stdio.h>
#include <string.h>
#include <AudioModuleSerialLib.h>

#ifdef _WIN32
#define waitKeyPressed() getch()
#define KeyPressed() GetAsyncKeyState(VK_ESCAPE)
#else
#define waitKeyPressed() readch()
#define KeyPressed() kbhit()
#define endl "\r\n"
#endif

using namespace std;

#define Audio_Module_ADDR	50
#define COM_BAUDRATE  921600

int main(int argc, char **argv)
{
#ifndef CCOMPORT_NO_QT
    QCoreApplication App(argc, argv);
#endif

    int Com_N;
#ifndef _WIN32
    init_keyboard();
#endif

    TAudioStatus AudioStatusInstance;
    AudioModuleSerialLib *AudioSL = new AudioModuleSerialLib();
    CCronometer SLocTimer;

    unsigned char presentationString[100];

    cout << endl << "Starting Open.Audio Acoustics demo" << endl;
    cout << "Please connect the board to the PC via USB Cable" << endl;
    cout << "Insert the COM Port number assigned to the board and press Enter:" << endl;
#ifdef _WIN32
    cout << "COM";
#else
    cout << "TTYACM";
#endif
    cin >> Com_N;

    cout << "Setting port and address...";
    AudioSL->SetCOMPortNumber(Com_N);
    AudioSL->SetClientAddress(1);
    cout << " OK" << endl;

#ifdef _WIN32
    cout << "Opening COM" << Com_N << "...";
#else
    cout << "Opening TTYACM" << Com_N << "...";
#endif
    if(AudioSL->Open(COM_BAUDRATE)<0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    cout << " OK" << endl << endl;

    cout << "Pinging Module... ";
    if(AudioSL->ASTCmd_Ping(Audio_Module_ADDR)<0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    else
    {
        cout << " OK" << endl << endl;
    }


    cout << "Device presentation string:" << endl;
    AudioSL->ASTCmd_GetPres(Audio_Module_ADDR,100,presentationString);
    cout << presentationString << endl << endl;

    /*Get application status*/
    cout << "Device status will be retrieved" << endl;
    cout << "Press any key to continue..." << endl;
    waitKeyPressed();

    if(AudioSL->AudioModuleCmd_GetStatus(Audio_Module_ADDR,0xFF,&AudioStatusInstance) != 0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    else
    {
        cout << " OK" << endl << endl;
    }

    /*Showing status of the running algorithms*/
    cout << "Current Algorithms running:" << endl;
    if(AudioStatusInstance.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_BF)
    {
        cout << "Beamforming, Type: " << (int)AudioStatusInstance.BeamStatus.Type << ", Direction: " << (int)AudioStatusInstance.BeamStatus.Direction <<endl;
    }
    if(AudioStatusInstance.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_SL)
    {
        cout << "Source Localization, Algorithm " << (int)AudioStatusInstance.SLocStatus.Algorithm << ", Resolution: " << (int)AudioStatusInstance.SLocStatus.Resolution <<endl;
    }
    if(AudioStatusInstance.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_EC)
    {
        cout << "AEC, Denoiser State: " << (int)AudioStatusInstance.AECStatus.Denoiser <<endl ;
    }
    if(AudioStatusInstance.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_DB)
    {
        cout << "dB SPL Estimation " <<endl ;
    }
    if(AudioStatusInstance.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_ASR)
    {
        cout << "Automatic Speech Recognition " << endl ;
    }

    cout << endl << "DEMO 1: Beamforming" << endl;
    cout << "Only Beamforming will be enabled" << endl;
    cout << "Press any key to continue..." << endl;
    waitKeyPressed();

    AudioStatusInstance.GeneralStatus.AlgorithmActivation = ALGO_ACTIVATION_BF;
    if(AudioSL->AudioModuleCmd_SetStatus(Audio_Module_ADDR, DOMAIN_GENERAL, &AudioStatusInstance)!=0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    else
    {
        cout << " OK" << endl << endl;
    }

    cout << "Direction will be set to 3" << endl;
    cout << "Press any key to continue..." << endl;
    waitKeyPressed();
    AudioStatusInstance.BeamStatus.Direction = 3;
    if(AudioSL->AudioModuleCmd_SetStatus(Audio_Module_ADDR, DOMAIN_BEAMFORMING, &AudioStatusInstance)!=0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    else
    {
        cout << " OK" << endl << endl;
    }

    cout << "DEMO 2: AEC" << endl;
    cout << "Only AEC will be enabled" << endl;
    cout << "Press any key to continue..." << endl;
    waitKeyPressed();

    AudioStatusInstance.GeneralStatus.AlgorithmActivation = ALGO_ACTIVATION_EC;
    if(AudioSL->AudioModuleCmd_SetStatus(Audio_Module_ADDR, DOMAIN_GENERAL, &AudioStatusInstance)!=0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    else
    {
        cout << " OK" << endl << endl;
    }

    cout << "DEMO 3: SLoc" << endl;
    cout << "Only SLoc will be enabled" << endl;
    cout << "Press any key to continue..." << endl;
    waitKeyPressed();

    AudioStatusInstance.GeneralStatus.AlgorithmActivation = ALGO_ACTIVATION_SL;
    if(AudioSL->AudioModuleCmd_SetStatus(Audio_Module_ADDR, DOMAIN_GENERAL, &AudioStatusInstance)!=0)
    {
        cout << " ERROR" << endl;
        return -1;
    }
    else
    {
        cout << " OK" << endl << endl;
    }

#ifdef _WIN32
cout << "Hit ESC key to stop angle requests"  << endl;
#else
cout << "Hit any key to stop angle requests"  << endl;
#endif
    SLocTimer.Reset();

    while(!KeyPressed())
    {
        double CurrentTime = SLocTimer.CurrentTime();
        if(CurrentTime >= 0.5)
        {
            if(AudioSL->AudioModuleCmd_GetStatus(Audio_Module_ADDR,DOMAIN_SLOC,&AudioStatusInstance) != 0)
            {
                cout << " ERROR" << endl;
                return -1;
            }
            else
            {
                cout <<  "Detected Angle: "  << (int)AudioStatusInstance.SLocStatus.Angle << "     \r";
            }
            SLocTimer.Reset();
        }
    }

#ifdef _WIN32
    cout << "\nClosing COM" << Com_N << "...";
#else
    cout << "\nClosing TTYACM" << Com_N << "...";
#endif

    if(AudioSL->Close()<0) {
        cout << " ERROR" << endl;
        return -1;
    }

    cout << endl << "Demo completed" << endl << "Press any key to continue..." << endl;
    waitKeyPressed();

#ifndef _WIN32
    close_keyboard();
#endif

    return 0;
}

