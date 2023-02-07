 // C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <cmath>
#include <stdlib.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function0
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/select.h>
#include <termio.h>


typedef unsigned int 	WORD;
typedef unsigned char   BYTE;


int _port ;
int _dir = 0;

unsigned char _bps_Set[7];
unsigned char _Nom_Vel_Set[8];
unsigned char _Vel_Ctrl_Set[10];
unsigned char _set_dir[7];
unsigned char _vel_cmd_dir0[10];
unsigned char _vel_cmd_dir1[10];
unsigned char _vel_cmd_zero[10];
unsigned char _Resol_Set[8];
unsigned char _Reset[6];
unsigned char _con_on[7];
unsigned char _con_off[7]; 
unsigned char _set_id[7];
unsigned char _id_5_dir0[10];

unsigned char _buf[255];
unsigned char _dummy_q;


int main()
{
  for(int i = 0 ; i < 255 ; i++)
  {
    _buf[i] = 0;
  }

  _bps_Set[0] = 0xFF; // Set Baudrate 115200 
  _bps_Set[1] = 0xFE;
  _bps_Set[2] = 0x00;
  _bps_Set[3] = 0x03;
  _bps_Set[5] = 0x07;
  _bps_Set[6] = 0x0D;
  _bps_Set[4] = 0xE8;

  _Nom_Vel_Set[0] = 0xFF; // 정격속도 설정
  _Nom_Vel_Set[1] = 0xFE;
  _Nom_Vel_Set[2] = 0x00;
  _Nom_Vel_Set[3] = 0x04;
  _Nom_Vel_Set[4] = 0x43;
  _Nom_Vel_Set[5] = 0x09;
  _Nom_Vel_Set[6] = 0x19;
  _Nom_Vel_Set[7] = 0x96;

  // _Vel_Ctrl_Set[0] = 0xFF; // 속도 제어기 설정
  // _Vel_Ctrl_Set[1] = 0xFE;
  // _Vel_Ctrl_Set[2] = 0x00;
  // _Vel_Ctrl_Set[3] = 0x06;
  // _Vel_Ctrl_Set[4] = 0x96;
  // _Vel_Ctrl_Set[5] = 0x05;
  // _Vel_Ctrl_Set[6] = 0xFE;
  // _Vel_Ctrl_Set[7] = 0xFE;
  // _Vel_Ctrl_Set[8] = 0x4E;
  // _Vel_Ctrl_Set[9] = 0x04;

  _Vel_Ctrl_Set[0] = 0xFF; // 속도 제어기 설정
  _Vel_Ctrl_Set[1] = 0xFE;
  _Vel_Ctrl_Set[2] = 0x00;
  _Vel_Ctrl_Set[3] = 0x06;
  _Vel_Ctrl_Set[4] = 0xDA;
  _Vel_Ctrl_Set[5] = 0x05;
  _Vel_Ctrl_Set[6] = 0xFE;
  _Vel_Ctrl_Set[7] = 0xFE;
  _Vel_Ctrl_Set[8] = 0xFE;
  _Vel_Ctrl_Set[9] = 0x20;

  _set_dir[0] = 0xFF; // 방향 설정
  _set_dir[1] = 0xFE;
  _set_dir[2] = 0x00;
  _set_dir[3] = 0x03;
  _set_dir[4] = 0xED;
  _set_dir[5] = 0x0E;
  _set_dir[6] = 0x01;


  ///////////////////////////////////////////////////////////////////////////////////////////////////
  
  _vel_cmd_dir0[0] = 0xFF;  // 0번방향으로 회전
  _vel_cmd_dir0[1] = 0xFE;
  _vel_cmd_dir0[2] = 0x00;
  _vel_cmd_dir0[3] = 0x06;
  _vel_cmd_dir0[4] = 0xF1;
  _vel_cmd_dir0[5] = 0x03;
  _vel_cmd_dir0[6] = 0x00;
  _vel_cmd_dir0[7] = 0xF6;
  _vel_cmd_dir0[8] = 0x05;
  _vel_cmd_dir0[9] = 0x0A;

  _id_5_dir0[0] = 0xFF;  // 0번방향으로 회전
  _id_5_dir0[1] = 0xFE;
  _id_5_dir0[2] = 0x05;
  _id_5_dir0[3] = 0x06;
  _id_5_dir0[4] = 0xEC;
  _id_5_dir0[5] = 0x03;
  _id_5_dir0[6] = 0x00;
  _id_5_dir0[7] = 0xF6;
  _id_5_dir0[8] = 0x05;
  _id_5_dir0[9] = 0x0A;

  _vel_cmd_dir1[0] = 0xFF;  // 1번방향으로 회전
  _vel_cmd_dir1[1] = 0xFE;
  _vel_cmd_dir1[2] = 0x00;
  _vel_cmd_dir1[3] = 0x06;
  _vel_cmd_dir1[4] = 0xF0;
  _vel_cmd_dir1[5] = 0x03;
  _vel_cmd_dir1[6] = 0x01;
  _vel_cmd_dir1[7] = 0xF6;
  _vel_cmd_dir1[8] = 0x05;
  _vel_cmd_dir1[9] = 0x0A;

  _vel_cmd_zero[0] = 0xFF;  // 정지
  _vel_cmd_zero[1] = 0xFE;
  _vel_cmd_zero[2] = 0x00;
  _vel_cmd_zero[3] = 0x06;
  _vel_cmd_zero[4] = 0xE2;//0x01; //check sum
  _vel_cmd_zero[5] = 0x03;
  _vel_cmd_zero[6] = 0x00;//0x01; // 방향
  _vel_cmd_zero[7] = 0x00;//0x02; // 속도1
  _vel_cmd_zero[8] = 0x00; // 속도2
  _vel_cmd_zero[9] = 0x14;//0x0A; // 도달 time(0.1s 단위)


  ///////////////////////////////////////////////////////////////////////////////////////////////////////



  _Resol_Set[0] = 0xFF; // 분해능 설정
  _Resol_Set[1] = 0xFE;
  _Resol_Set[2] = 0x00;
  _Resol_Set[3] = 0x04;
  _Resol_Set[4] = 0xEF;
  _Resol_Set[5] = 0x0A;
  _Resol_Set[6] = 0x04;
  _Resol_Set[7] = 0x00;

  _Reset[0] = 0xFF; // 리셋
  _Reset[1] = 0xFE;
  _Reset[2] = 0xFF;
  _Reset[3] = 0x02;
  _Reset[4] = 0xF1;
  _Reset[5] = 0x0D;

  _con_off[0] = 0xFF; // 제어 off
  _con_off[1] = 0xFE;
  _con_off[2] = 0x00;
  _con_off[3] = 0x03;
  _con_off[4] = 0xEF;
  _con_off[5] = 0x0C;
  _con_off[6] = 0x01;

  _con_on[0] = 0xFF; // 제어 on
  _con_on[1] = 0xFE;
  _con_on[2] = 0x00;
  _con_on[3] = 0x03;
  _con_on[4] = 0xF0;
  _con_on[5] = 0x0C;
  _con_on[6] = 0x00;

  _set_id[0] = 0xFF;
  _set_id[1] = 0xFE;
  _set_id[2] = 0x00;
  _set_id[3] = 0x03;
  _set_id[4] = 0xF1;
  _set_id[5] = 0x06;
  _set_id[6] = 0x05;

  _dummy_q = 'q';

  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)



////////////////////////////////////////////////
////////////////////////////////////////////////
  _port = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY);
///////////// 연결된 포트번호로 변경 ////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////




  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  bzero(&tty, sizeof(tty)); 
  // Read in existing settings, and handle any error
  if(tcgetattr(_port, &tty) != 0) 
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  //tty.c_iflag = INPCK;
  //tty.c_oflag = 0;
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  

  //cfsetospeed(&tty, B9600);
  //cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B115200); // Baudrate 115200
  cfsetispeed(&tty, B115200);

  tty.c_lflag = 0;
  bzero(tty.c_cc, NCCS);
  tty.c_cc[VTIME] = 0; 
  tty.c_cc[VMIN] = 1;  

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CS8;
  tty.c_lflag &= ~ECHO;  
  tty.c_oflag = 0;
  //tty.c_lflag  =  ICANON;
  
  tcsetattr(_port,  TCSANOW,  &tty);
  tcflush(_port,  TCIOFLUSH);//
  // Save tty settings, also checking for error
  if (tcsetattr(_port, TCSANOW, &tty) != 0) 
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  
  // write(_port, _Reset,sizeof(_Reset)); // 리셋
  // std::cout << "Reset" <<std::endl;
  // usleep(20000);
  // tcflush(_port,  TCIOFLUSH);

  //write(_port, _bps_Set,sizeof(_bps_Set)); // bps 설정 (-> 115200)
  //std::cout << "Baudrate Setting: 115200bps complete." <<std::endl;
  //usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);

  //  write(_port, _con_off,sizeof(_con_off));
  // usleep(200000);
  // tcflush(_port,  TCIOFLUSH);
  // // std::cout << "방향 = 0 " << std::endl;
  // usleep(50000);
  
  write(_port, _con_on,sizeof(_con_on));
  usleep(200000);
  tcflush(_port,  TCIOFLUSH);
  usleep(50000);
  tcflush(_port,  TCIOFLUSH);
  
  write(_port, _Nom_Vel_Set,sizeof(_Nom_Vel_Set)); // 정격속도 설정
  std::cout << "Nominal Velocity Setting complete." <<std::endl;
  usleep(20000);
  tcflush(_port,  TCIOFLUSH);
  usleep(2000);

  write(_port, _set_dir,sizeof(_set_dir)); // 제어방향 설정

  write(_port, _Vel_Ctrl_Set,sizeof(_Vel_Ctrl_Set)); // 속도 제어기 설정
  std::cout << "Velocity Controller Setting complete." <<std::endl;
  usleep(20000);
  tcflush(_port,  TCIOFLUSH);
  usleep(2000);

  write(_port, _Resol_Set,sizeof(_Resol_Set)); // 분해능 설정
  std::cout << "Resolution Setting Complete." <<std::endl;
  usleep(20000);
  tcflush(_port,  TCIOFLUSH);
  usleep(2000);

  write(_port, _con_on,sizeof(_con_on));

  usleep(20000);











  while(1)
  {
   
    //write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_dir0)); // 0번방향으로 회전
    //write(_port, _vel_cmd_dir1,sizeof(_vel_cmd_dir1)); // 1번방향으로 회전
    //write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero)); // 정지
    //std::cout << "- Now Loading -" <<std::endl;

    write(_port, _id_5_dir0,sizeof(_id_5_dir0)); // 0번방향으로 회전
    

    //write(_port, _set_id,sizeof(_set_id));



    usleep(20000);
  }
 
}
