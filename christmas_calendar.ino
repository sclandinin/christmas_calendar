/***********************************************************************************
Christmas Calendar - By Scott Clandinin 2020
In another program, set the current time of your RTC. DST is not taken into account
in the following code.
**********************************************************************************/

#include <SD.h>
#include <SPI.h>
#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_KBV.h> //Hardware-specific library
#include <Wire.h>
#include "RTClib.h"

DS1307 rtc; //using DS3231 but this setting operates the device nearly twice as fast

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV my_lcd(ILI9486,A3,A2,A1,A0,A4); //model,cs,cd,wr,rd,reset

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define PIXEL_NUMBER  (my_lcd.Get_Display_Width()/4)
#define FILE_NUMBER_A 128
#define FILE_NUMBER_B 128
#define FILE_NUMBER_C 110
#define FILE_NAME_SIZE_MAX 8



uint32_t bmp_offset = 0;
uint16_t s_width = my_lcd.Get_Display_Width();  
uint16_t s_heigh = my_lcd.Get_Display_Height();
//int16_t PIXEL_NUMBER;

char file_name_A[FILE_NUMBER_A][FILE_NAME_SIZE_MAX];
char file_name_B[FILE_NUMBER_B][FILE_NAME_SIZE_MAX];
char file_name_C[FILE_NUMBER_C][FILE_NAME_SIZE_MAX];


uint16_t read_16(File fp)
{
    uint8_t low;
    uint16_t high;
    low = fp.read();
    high = fp.read();
    return (high<<8)|low;
}

uint32_t read_32(File fp)
{
    uint16_t low;
    uint32_t high;
    low = read_16(fp);
    high = read_16(fp);
    return (high<<8)|low;   
 }
 
bool analysis_bpm_header(File fp)
{
    if(read_16(fp) != 0x4D42)
    {
      return false;  
    }
    //get bpm size
    read_32(fp);
    //get creator information
    read_32(fp);
    //get offset information
    bmp_offset = read_32(fp);
    //get DIB infomation
    read_32(fp);
    //get width and heigh information
    uint32_t bpm_width = read_32(fp);
    uint32_t bpm_heigh = read_32(fp);
    if((bpm_width != s_width) || (bpm_heigh != s_heigh))
    {
      return false; 
    }
    if(read_16(fp) != 1)
    {
        return false;
    }
    read_16(fp);
    if(read_32(fp) != 0)
    {
      return false; 
     }
     return true;
}

void draw_bmp_picture(File fp)
{
  uint16_t i,j,k,l,m=0;
  uint8_t bpm_data[PIXEL_NUMBER*3] = {0};
  uint16_t bpm_color[PIXEL_NUMBER];
  fp.seek(bmp_offset);
  for(i = 0;i < s_heigh;i++)
  {
    for(j = 0;j<s_width/PIXEL_NUMBER;j++)
    {
      m = 0;
      fp.read(bpm_data,PIXEL_NUMBER*3);
      for(k = 0;k<PIXEL_NUMBER;k++)
      {
        bpm_color[k]= my_lcd.Color_To_565(bpm_data[m+2], bpm_data[m+1], bpm_data[m+0]); //change to 565
        m +=3;
      }
      for(l = 0;l<PIXEL_NUMBER;l++)
      {
        my_lcd.Set_Draw_color(bpm_color[l]);
        my_lcd.Draw_Pixel(j*PIXEL_NUMBER+l,i);
      }    
     }
   }    
}

void setup() 
{
  
  Serial.begin(57600);
  Serial.println("set up");
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
    pinMode(53, OUTPUT); //SS defined as output, needed for SPI to work on MEGA
}

void loop() 
{


    Serial.println("loop init");
    //start initialization
    my_lcd.Init_LCD();
    Serial.println(my_lcd.Read_ID(), HEX);
    my_lcd.Fill_Screen(BLUE);

    strcpy(file_name_A[0],"0.bmp");
    strcpy(file_name_A[1],"1.bmp");
    strcpy(file_name_A[2],"2.bmp");
    strcpy(file_name_A[3],"3.bmp");
    strcpy(file_name_A[4],"4.bmp");
    strcpy(file_name_A[5],"5.bmp");
    strcpy(file_name_A[6],"6.bmp");
    strcpy(file_name_A[7],"7.bmp");
    strcpy(file_name_A[8],"8.bmp");
    strcpy(file_name_A[9],"9.bmp");
    strcpy(file_name_A[10],"10.bmp");
    strcpy(file_name_A[11],"11.bmp");
    strcpy(file_name_A[12],"12.bmp");
    strcpy(file_name_A[13],"13.bmp");
    strcpy(file_name_A[14],"14.bmp");
    strcpy(file_name_A[15],"15.bmp");
    strcpy(file_name_A[16],"16.bmp");
    strcpy(file_name_A[17],"17.bmp");
    strcpy(file_name_A[18],"18.bmp");
    strcpy(file_name_A[19],"19.bmp");
    strcpy(file_name_A[20],"20.bmp");
    strcpy(file_name_A[21],"21.bmp");
    strcpy(file_name_A[22],"22.bmp");
    strcpy(file_name_A[23],"23.bmp");
    strcpy(file_name_A[24],"24.bmp");
    strcpy(file_name_A[25],"25.bmp");
    strcpy(file_name_A[26],"26.bmp");
    strcpy(file_name_A[27],"27.bmp");
    strcpy(file_name_A[28],"28.bmp");
    strcpy(file_name_A[29],"29.bmp");
    strcpy(file_name_A[30],"30.bmp");
    strcpy(file_name_A[31],"31.bmp");
    strcpy(file_name_A[32],"32.bmp");
    strcpy(file_name_A[33],"33.bmp");
    strcpy(file_name_A[34],"34.bmp");
    strcpy(file_name_A[35],"35.bmp");
    strcpy(file_name_A[36],"36.bmp");
    strcpy(file_name_A[37],"37.bmp");
    strcpy(file_name_A[38],"38.bmp");
    strcpy(file_name_A[39],"39.bmp");
    strcpy(file_name_A[40],"40.bmp");
    strcpy(file_name_A[41],"41.bmp");
    strcpy(file_name_A[42],"42.bmp");
    strcpy(file_name_A[43],"43.bmp");
    strcpy(file_name_A[44],"44.bmp");
    strcpy(file_name_A[45],"45.bmp");
    strcpy(file_name_A[46],"46.bmp");
    strcpy(file_name_A[47],"47.bmp");
    strcpy(file_name_A[48],"48.bmp");
    strcpy(file_name_A[49],"49.bmp");
    strcpy(file_name_A[50],"50.bmp");
    strcpy(file_name_A[51],"51.bmp");
    strcpy(file_name_A[52],"52.bmp");
    strcpy(file_name_A[53],"53.bmp");
    strcpy(file_name_A[54],"54.bmp");
    strcpy(file_name_A[55],"55.bmp");
    strcpy(file_name_A[56],"56.bmp");
    strcpy(file_name_A[57],"57.bmp");
    strcpy(file_name_A[58],"58.bmp");
    strcpy(file_name_A[59],"59.bmp");
    strcpy(file_name_A[60],"60.bmp");
    strcpy(file_name_A[61],"61.bmp");
    strcpy(file_name_A[62],"62.bmp");
    strcpy(file_name_A[63],"63.bmp");
    strcpy(file_name_A[64],"64.bmp");
    strcpy(file_name_A[65],"65.bmp");
    strcpy(file_name_A[66],"66.bmp");
    strcpy(file_name_A[67],"67.bmp");
    strcpy(file_name_A[68],"68.bmp");
    strcpy(file_name_A[69],"69.bmp");
    strcpy(file_name_A[70],"70.bmp");
    strcpy(file_name_A[71],"71.bmp");
    strcpy(file_name_A[72],"72.bmp");
    strcpy(file_name_A[73],"73.bmp");
    strcpy(file_name_A[74],"74.bmp");
    strcpy(file_name_A[75],"75.bmp");
    strcpy(file_name_A[76],"76.bmp");
    strcpy(file_name_A[77],"77.bmp");
    strcpy(file_name_A[78],"78.bmp");
    strcpy(file_name_A[79],"79.bmp");
    strcpy(file_name_A[80],"80.bmp");
    strcpy(file_name_A[81],"81.bmp");
    strcpy(file_name_A[82],"82.bmp");
    strcpy(file_name_A[83],"83.bmp");
    strcpy(file_name_A[84],"84.bmp");
    strcpy(file_name_A[85],"85.bmp");
    strcpy(file_name_A[86],"86.bmp");
    strcpy(file_name_A[87],"87.bmp");
    strcpy(file_name_A[88],"88.bmp");
    strcpy(file_name_A[89],"89.bmp");
    strcpy(file_name_A[90],"90.bmp");
    strcpy(file_name_A[91],"91.bmp");
    strcpy(file_name_A[92],"92.bmp");
    strcpy(file_name_A[93],"93.bmp");
    strcpy(file_name_A[94],"94.bmp");
    strcpy(file_name_A[95],"95.bmp");
    strcpy(file_name_A[96],"96.bmp");
    strcpy(file_name_A[97],"97.bmp");
    strcpy(file_name_A[98],"98.bmp");
    strcpy(file_name_A[99],"99.bmp");
    strcpy(file_name_A[100],"100.bmp");
    strcpy(file_name_A[101],"101.bmp");
    strcpy(file_name_A[102],"102.bmp");
    strcpy(file_name_A[103],"103.bmp");
    strcpy(file_name_A[104],"104.bmp");
    strcpy(file_name_A[105],"105.bmp");
    strcpy(file_name_A[106],"106.bmp");
    strcpy(file_name_A[107],"107.bmp");
    strcpy(file_name_A[108],"108.bmp");
    strcpy(file_name_A[109],"109.bmp");
    strcpy(file_name_A[110],"110.bmp");
    strcpy(file_name_A[111],"111.bmp");
    strcpy(file_name_A[112],"112.bmp");
    strcpy(file_name_A[113],"113.bmp");
    strcpy(file_name_A[114],"114.bmp");
    strcpy(file_name_A[115],"115.bmp");
    strcpy(file_name_A[116],"116.bmp");
    strcpy(file_name_A[117],"117.bmp");
    strcpy(file_name_A[118],"118.bmp");
    strcpy(file_name_A[119],"119.bmp");
    strcpy(file_name_A[120],"120.bmp");
    strcpy(file_name_A[121],"121.bmp");
    strcpy(file_name_A[122],"122.bmp");
    strcpy(file_name_A[123],"123.bmp");
    strcpy(file_name_A[124],"124.bmp");
    strcpy(file_name_A[125],"125.bmp");
    strcpy(file_name_A[126],"126.bmp");
    strcpy(file_name_A[127],"127.bmp");

    strcpy(file_name_B[0],"128.bmp");
    strcpy(file_name_B[1],"129.bmp");
    strcpy(file_name_B[2],"130.bmp");
    strcpy(file_name_B[3],"131.bmp");
    strcpy(file_name_B[4],"132.bmp");
    strcpy(file_name_B[5],"133.bmp");
    strcpy(file_name_B[6],"134.bmp");
    strcpy(file_name_B[7],"135.bmp");
    strcpy(file_name_B[8],"136.bmp");
    strcpy(file_name_B[9],"137.bmp");
    strcpy(file_name_B[10],"138.bmp");
    strcpy(file_name_B[11],"139.bmp");
    strcpy(file_name_B[12],"140.bmp");
    strcpy(file_name_B[13],"141.bmp");
    strcpy(file_name_B[14],"142.bmp");
    strcpy(file_name_B[15],"143.bmp");
    strcpy(file_name_B[16],"144.bmp");
    strcpy(file_name_B[17],"145.bmp");
    strcpy(file_name_B[18],"146.bmp");
    strcpy(file_name_B[19],"147.bmp");
    strcpy(file_name_B[20],"148.bmp");
    strcpy(file_name_B[21],"149.bmp");
    strcpy(file_name_B[22],"150.bmp");
    strcpy(file_name_B[23],"151.bmp");
    strcpy(file_name_B[24],"152.bmp");
    strcpy(file_name_B[25],"153.bmp");
    strcpy(file_name_B[26],"154.bmp");
    strcpy(file_name_B[27],"155.bmp");
    strcpy(file_name_B[28],"156.bmp");
    strcpy(file_name_B[29],"157.bmp");
    strcpy(file_name_B[30],"158.bmp");
    strcpy(file_name_B[31],"159.bmp");
    strcpy(file_name_B[32],"160.bmp");
    strcpy(file_name_B[33],"161.bmp");
    strcpy(file_name_B[34],"162.bmp");
    strcpy(file_name_B[35],"163.bmp");
    strcpy(file_name_B[36],"164.bmp");
    strcpy(file_name_B[37],"165.bmp");
    strcpy(file_name_B[38],"166.bmp");
    strcpy(file_name_B[39],"167.bmp");
    strcpy(file_name_B[40],"168.bmp");
    strcpy(file_name_B[41],"169.bmp");
    strcpy(file_name_B[42],"170.bmp");
    strcpy(file_name_B[43],"171.bmp");
    strcpy(file_name_B[44],"172.bmp");
    strcpy(file_name_B[45],"173.bmp");
    strcpy(file_name_B[46],"174.bmp");
    strcpy(file_name_B[47],"175.bmp");
    strcpy(file_name_B[48],"176.bmp");
    strcpy(file_name_B[49],"177.bmp");
    strcpy(file_name_B[50],"178.bmp");
    strcpy(file_name_B[51],"179.bmp");
    strcpy(file_name_B[52],"180.bmp");
    strcpy(file_name_B[53],"181.bmp");
    strcpy(file_name_B[54],"182.bmp");
    strcpy(file_name_B[55],"183.bmp");
    strcpy(file_name_B[56],"184.bmp");
    strcpy(file_name_B[57],"185.bmp");
    strcpy(file_name_B[58],"186.bmp");
    strcpy(file_name_B[59],"187.bmp");
    strcpy(file_name_B[60],"188.bmp");
    strcpy(file_name_B[61],"189.bmp");
    strcpy(file_name_B[62],"190.bmp");
    strcpy(file_name_B[63],"191.bmp");
    strcpy(file_name_B[64],"192.bmp");
    strcpy(file_name_B[65],"193.bmp");
    strcpy(file_name_B[66],"194.bmp");
    strcpy(file_name_B[67],"195.bmp");
    strcpy(file_name_B[68],"196.bmp");
    strcpy(file_name_B[69],"197.bmp");
    strcpy(file_name_B[70],"198.bmp");
    strcpy(file_name_B[71],"199.bmp");
    strcpy(file_name_B[72],"200.bmp");
    strcpy(file_name_B[73],"201.bmp");
    strcpy(file_name_B[74],"202.bmp");
    strcpy(file_name_B[75],"203.bmp");
    strcpy(file_name_B[76],"204.bmp");
    strcpy(file_name_B[77],"205.bmp");
    strcpy(file_name_B[78],"206.bmp");
    strcpy(file_name_B[79],"207.bmp");
    strcpy(file_name_B[80],"208.bmp");
    strcpy(file_name_B[81],"209.bmp");
    strcpy(file_name_B[82],"210.bmp");
    strcpy(file_name_B[83],"211.bmp");
    strcpy(file_name_B[84],"212.bmp");
    strcpy(file_name_B[85],"213.bmp");
    strcpy(file_name_B[86],"214.bmp");
    strcpy(file_name_B[87],"215.bmp");
    strcpy(file_name_B[88],"216.bmp");
    strcpy(file_name_B[89],"217.bmp");
    strcpy(file_name_B[90],"218.bmp");
    strcpy(file_name_B[91],"219.bmp");
    strcpy(file_name_B[92],"220.bmp");
    strcpy(file_name_B[93],"221.bmp");
    strcpy(file_name_B[94],"222.bmp");
    strcpy(file_name_B[95],"223.bmp");
    strcpy(file_name_B[96],"224.bmp");
    strcpy(file_name_B[97],"225.bmp");
    strcpy(file_name_B[98],"226.bmp");
    strcpy(file_name_B[99],"227.bmp");
    strcpy(file_name_B[100],"228.bmp");
    strcpy(file_name_B[101],"229.bmp");
    strcpy(file_name_B[102],"230.bmp");
    strcpy(file_name_B[103],"231.bmp");
    strcpy(file_name_B[104],"232.bmp");
    strcpy(file_name_B[105],"233.bmp");
    strcpy(file_name_B[106],"234.bmp");
    strcpy(file_name_B[107],"235.bmp");
    strcpy(file_name_B[108],"236.bmp");
    strcpy(file_name_B[109],"237.bmp");
    strcpy(file_name_B[110],"238.bmp");
    strcpy(file_name_B[111],"239.bmp");
    strcpy(file_name_B[112],"240.bmp");
    strcpy(file_name_B[113],"241.bmp");
    strcpy(file_name_B[114],"242.bmp");
    strcpy(file_name_B[115],"243.bmp");
    strcpy(file_name_B[116],"244.bmp");
    strcpy(file_name_B[117],"245.bmp");
    strcpy(file_name_B[118],"246.bmp");
    strcpy(file_name_B[119],"247.bmp");
    strcpy(file_name_B[120],"248.bmp");
    strcpy(file_name_B[121],"249.bmp");
    strcpy(file_name_B[122],"250.bmp");
    strcpy(file_name_B[123],"251.bmp");
    strcpy(file_name_B[124],"252.bmp");
    strcpy(file_name_B[125],"253.bmp");
    strcpy(file_name_B[126],"254.bmp");
    strcpy(file_name_B[127],"255.bmp");

    strcpy(file_name_C[0],"256.bmp");
    strcpy(file_name_C[1],"257.bmp");
    strcpy(file_name_C[2],"258.bmp");
    strcpy(file_name_C[3],"259.bmp");
    strcpy(file_name_C[4],"260.bmp");
    strcpy(file_name_C[5],"261.bmp");
    strcpy(file_name_C[6],"262.bmp");
    strcpy(file_name_C[7],"263.bmp");
    strcpy(file_name_C[8],"264.bmp");
    strcpy(file_name_C[9],"265.bmp");
    strcpy(file_name_C[10],"266.bmp");
    strcpy(file_name_C[11],"267.bmp");
    strcpy(file_name_C[12],"268.bmp");
    strcpy(file_name_C[13],"269.bmp");
    strcpy(file_name_C[14],"270.bmp");
    strcpy(file_name_C[15],"271.bmp");
    strcpy(file_name_C[16],"272.bmp");
    strcpy(file_name_C[17],"273.bmp");
    strcpy(file_name_C[18],"274.bmp");
    strcpy(file_name_C[19],"275.bmp");
    strcpy(file_name_C[20],"276.bmp");
    strcpy(file_name_C[21],"277.bmp");
    strcpy(file_name_C[22],"278.bmp");
    strcpy(file_name_C[23],"279.bmp");
    strcpy(file_name_C[24],"280.bmp");
    strcpy(file_name_C[25],"281.bmp");
    strcpy(file_name_C[26],"282.bmp");
    strcpy(file_name_C[27],"283.bmp");
    strcpy(file_name_C[28],"284.bmp");
    strcpy(file_name_C[29],"285.bmp");
    strcpy(file_name_C[30],"286.bmp");
    strcpy(file_name_C[31],"287.bmp");
    strcpy(file_name_C[32],"288.bmp");
    strcpy(file_name_C[33],"289.bmp");
    strcpy(file_name_C[34],"290.bmp");
    strcpy(file_name_C[35],"291.bmp");
    strcpy(file_name_C[36],"292.bmp");
    strcpy(file_name_C[37],"293.bmp");
    strcpy(file_name_C[38],"294.bmp");
    strcpy(file_name_C[39],"295.bmp");
    strcpy(file_name_C[40],"296.bmp");
    strcpy(file_name_C[41],"297.bmp");
    strcpy(file_name_C[42],"298.bmp");
    strcpy(file_name_C[43],"299.bmp");
    strcpy(file_name_C[44],"300.bmp");
    strcpy(file_name_C[45],"301.bmp");
    strcpy(file_name_C[46],"302.bmp");
    strcpy(file_name_C[47],"303.bmp");
    strcpy(file_name_C[48],"304.bmp");
    strcpy(file_name_C[49],"305.bmp");
    strcpy(file_name_C[50],"306.bmp");
    strcpy(file_name_C[51],"307.bmp");
    strcpy(file_name_C[52],"308.bmp");
    strcpy(file_name_C[53],"309.bmp");
    strcpy(file_name_C[54],"310.bmp");
    strcpy(file_name_C[55],"311.bmp");
    strcpy(file_name_C[56],"312.bmp");
    strcpy(file_name_C[57],"313.bmp");
    strcpy(file_name_C[58],"314.bmp");
    strcpy(file_name_C[59],"315.bmp");
    strcpy(file_name_C[60],"316.bmp");
    strcpy(file_name_C[61],"317.bmp");
    strcpy(file_name_C[62],"318.bmp");
    strcpy(file_name_C[63],"319.bmp");
    strcpy(file_name_C[64],"320.bmp");
    strcpy(file_name_C[65],"321.bmp");
    strcpy(file_name_C[66],"322.bmp");
    strcpy(file_name_C[67],"323.bmp");
    strcpy(file_name_C[68],"324.bmp");
    strcpy(file_name_C[69],"325.bmp");
    strcpy(file_name_C[70],"326.bmp");
    strcpy(file_name_C[71],"327.bmp");
    strcpy(file_name_C[72],"328.bmp");
    strcpy(file_name_C[73],"329.bmp");
    strcpy(file_name_C[74],"330.bmp");
    strcpy(file_name_C[75],"331.bmp");
    strcpy(file_name_C[76],"332.bmp");
    strcpy(file_name_C[77],"333.bmp");
    strcpy(file_name_C[78],"334.bmp");
    strcpy(file_name_C[79],"335.bmp");
    strcpy(file_name_C[80],"336.bmp");
    strcpy(file_name_C[81],"337.bmp");
    strcpy(file_name_C[82],"338.bmp");
    strcpy(file_name_C[83],"339.bmp");
    strcpy(file_name_C[84],"340.bmp");
    strcpy(file_name_C[85],"341.bmp");
    strcpy(file_name_C[86],"342.bmp");
    strcpy(file_name_C[87],"343.bmp");
    strcpy(file_name_C[88],"344.bmp");
    strcpy(file_name_C[89],"345.bmp");
    strcpy(file_name_C[90],"346.bmp");
    strcpy(file_name_C[91],"347.bmp");
    strcpy(file_name_C[92],"348.bmp");
    strcpy(file_name_C[93],"349.bmp");
    strcpy(file_name_C[94],"350.bmp");
    strcpy(file_name_C[95],"351.bmp");
    strcpy(file_name_C[96],"352.bmp");
    strcpy(file_name_C[97],"353.bmp");
    strcpy(file_name_C[98],"354.bmp");
    strcpy(file_name_C[99],"355.bmp");
    strcpy(file_name_C[100],"356.bmp");
    strcpy(file_name_C[101],"357.bmp");
    strcpy(file_name_C[102],"358.bmp");
    strcpy(file_name_C[103],"359.bmp");
    strcpy(file_name_C[104],"360.bmp");
    strcpy(file_name_C[105],"361.bmp");
    strcpy(file_name_C[106],"362.bmp");
    strcpy(file_name_C[107],"363.bmp");
    strcpy(file_name_C[108],"364.bmp");
    strcpy(file_name_C[109],"365.bmp");


    //Init SD_Card
    pinMode(10, OUTPUT);
   
    if (!SD.begin(10)) 
    {
        my_lcd.Set_Text_Back_colour(BLUE);
        my_lcd.Set_Text_colour(WHITE);    
        my_lcd.Set_Text_Size(1);
        my_lcd.Print_String("SD Card Init fail!",0,0);
    }
    //end initialization

    //the loop of code that does the stuff
    do
    {
      int i = 0;
      long time_now = 0;
      long mins_until = 0;
      int days_until = 0;
      int month_guess = 0;
      int day_guess = 0;
      File bmp_file;


      Serial.println("do loop");
      //check RTC for days till Christmas
      DateTime now = rtc.now();
      
      time_now = now.unixtime(); //get time
      //calculate future date and time every 60s until we reach 12/25
      while (day_guess != 25 || month_guess != 12)
      {
        DateTime future (time_now);
        time_now = time_now + 60;
        mins_until = mins_until + 1;
        month_guess = future.month();
        day_guess = future.day();
      }
      //don't to the day/time adjust if it is actually Christmas
      time_now = now.unixtime(); //get time
      DateTime future (time_now);
      month_guess = future.month();
      day_guess = future.day();
      if (day_guess != 25 || month_guess != 12)
      {
        mins_until = mins_until - 1; //remove extra minute added in last operation
        days_until = (mins_until / 1440) + 1; //add extra day skipped forward
      }
      
      Serial.println("Days until Christmas:");
      Serial.println(days_until);
      Serial.println("Minutes until Christmas:");
      Serial.println(mins_until);  
      mins_until = 0;
      month_guess = 0;
      day_guess = 0;

 
      if (days_until > 255)
      {
        days_until = days_until - 256;
        bmp_file = SD.open(file_name_C[days_until]);
        if(!bmp_file)
        {
            my_lcd.Set_Text_Back_colour(BLUE);
            my_lcd.Set_Text_colour(WHITE);    
            my_lcd.Set_Text_Size(1);
            my_lcd.Print_String("didnt find BMPimage!",0,10);
            while(1);
        }
        if(!analysis_bpm_header(bmp_file))
        {  
            my_lcd.Set_Text_Back_colour(BLUE);
            my_lcd.Set_Text_colour(WHITE);    
            my_lcd.Set_Text_Size(1);
            my_lcd.Print_String("bad bmp picture!",0,0);
            return;
        }
        draw_bmp_picture(bmp_file);
        bmp_file.close(); 
        delay(100);
        days_until = days_until + 256;
      }
      
      else if (days_until < 128)
      {
        bmp_file = SD.open(file_name_A[days_until]);
        if(!bmp_file)
        {
            my_lcd.Set_Text_Back_colour(BLUE);
            my_lcd.Set_Text_colour(WHITE);    
            my_lcd.Set_Text_Size(1);
            my_lcd.Print_String("didnt find BMPimage!",0,10);
            while(1);
        }
        if(!analysis_bpm_header(bmp_file))
        {  
            my_lcd.Set_Text_Back_colour(BLUE);
            my_lcd.Set_Text_colour(WHITE);    
            my_lcd.Set_Text_Size(1);
            my_lcd.Print_String("bad bmp picture!",0,0);
            return;
        }
        draw_bmp_picture(bmp_file);
        bmp_file.close(); 
        delay(100);
      }

      else if (127 < days_until < 256)
      {
        days_until = days_until - 128;
        bmp_file = SD.open(file_name_B[days_until]);
        if(!bmp_file)
        {
            my_lcd.Set_Text_Back_colour(BLUE);
            my_lcd.Set_Text_colour(WHITE);    
            my_lcd.Set_Text_Size(1);
            my_lcd.Print_String("didnt find BMPimage!",0,10);
            while(1);
        }
        if(!analysis_bpm_header(bmp_file))
        {  
            my_lcd.Set_Text_Back_colour(BLUE);
            my_lcd.Set_Text_colour(WHITE);    
            my_lcd.Set_Text_Size(1);
            my_lcd.Print_String("bad bmp picture!",0,0);
            return;
        }
        draw_bmp_picture(bmp_file);
        bmp_file.close(); 
        delay(100);
        days_until = days_until + 128;
      }
 
    } while (1);
}
