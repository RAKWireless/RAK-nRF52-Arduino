
#include <PDM.h>
#include "SPI.h"
#include "SD.h"//http://librarymanager/All#SD

File myFile;
#define SAMPLE_SIZE   256
#define SAMPLE_PACKET_LENGTH 300
#define PCM_HEAD_SIZE 44
#define BIT_PER_SAMPLE 16
// default number of output channels
static const int channels = 2;
// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[SAMPLE_SIZE];
short totalBuffer[SAMPLE_PACKET_LENGTH][SAMPLE_SIZE];


#define PDM_DATA_PIN  21
#define PDM_CLK_PIN   4
#define PDM_PWR_PIN   -1

// Number of audio samples read
int samplesRead = 0;
int flag = 0;
String pcm_format = "RIFF";

void add_head()
{
  uint8_t tmp = 0;
  //RIFF CHUNK
  uint32_t total_size = SAMPLE_PACKET_LENGTH*SAMPLE_SIZE*2+PCM_HEAD_SIZE-8; 
  tmp = total_size & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (total_size & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (total_size & 0x00FF0000)>>16;  
  pcm_format += char(tmp);     
  tmp = (total_size & 0xFF000000)>>24;  
  pcm_format += char(tmp);

  pcm_format+="WAVE";      

  //Format CHUNK
  pcm_format+="fmt ";
  tmp = 16;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);
  pcm_format+=char(tmp);
  pcm_format+=char(tmp);
  tmp = 1;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);
  tmp = 2;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);
  
  uint32_t fre = frequency;  //little,
  tmp = fre & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (fre & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (fre & 0x00FF0000)>>16;  
  pcm_format += char(tmp);     
  tmp = (fre & 0xFF000000)>>24;  
  pcm_format += char(tmp);
  uint32_t ByteRate = frequency*channels*BIT_PER_SAMPLE/8;  //little, ByteRate is 32000, 播放速度换算成字节单位就是32000(字节/秒)。
  tmp = ByteRate & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (ByteRate & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (ByteRate & 0x00FF0000)>>16;  
  pcm_format += char(tmp);    
  tmp = (ByteRate & 0xFF000000)>>24;  
  pcm_format += char(tmp);  
  uint16_t BlockAlign = channels*BIT_PER_SAMPLE/8;      //little, BlockAlign is 4, here should be 0x0400
  tmp = BlockAlign & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (BlockAlign & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  
  tmp = 16;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);  


  //Data CHUNK
  pcm_format += "data";      
  uint32_t a_size = SAMPLE_PACKET_LENGTH*SAMPLE_SIZE*2; //little
  tmp = a_size & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (a_size & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (a_size & 0x00FF0000)>>16;  
  pcm_format += char(tmp);   
  tmp = (a_size & 0xFF000000)>>24;  
  pcm_format += char(tmp);   
  myFile.print(pcm_format);
  myFile.close(); 
  Serial.println("head finish!"); 
}

////little store
void store_data()
{
  uint8_t tmp = 0;
  File file;
  String stream = "";
  for(int i=0;i<SAMPLE_PACKET_LENGTH;i++)
  {
    stream = "";
    file = SD.open("record.wav", FILE_WRITE);    
    for(int j=0;j<SAMPLE_SIZE;j++)
    {
      tmp = totalBuffer[i][j] & 0x00FF;   
      stream += char(tmp);   
      tmp = (totalBuffer[i][j] & 0xFF00) >> 8;  
      stream += char(tmp);   
    }
    if(file)
    {
      Serial.print("Store line:");
      Serial.println(i);      
      file.print(stream);
      file.close();
    }
    else
    {
      Serial.println("fail to open");    
    }
  }
}

void setup() {
  pinMode(WB_IO2,OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  pinMode(PIN_LED1,OUTPUT);
  pinMode(PIN_LED2,OUTPUT);
  Serial.begin(115200);
  Serial.begin(115200);
  while (!Serial)
  if (!SD.begin()) 
  {    
    Serial.println("Card Mount Failed! Please make sure the card is inserted!");
    return;
  }

  // Check to see if the file exists:
  if (SD.exists("record.wav")) {
    Serial.println("Removing record.wav...");
    SD.remove("record.wav");    
  }
  delay(1000);
  // open a new file and immediately close it:
  Serial.println("Creating record.wav...");
  myFile = SD.open("record.wav", FILE_WRITE);
  add_head();
  
  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, PDM_PWR_PIN);
  //PDM.setBufferSize(256);//default is 512 uin8_t
  // configure the data receive callback
  PDM.onReceive(onPDMdata);
  Serial.println("Ready to say!");    
  delay(1000);
  digitalWrite(PIN_LED2, HIGH);
  Serial.println("3");  
  delay(1000);
  Serial.println("2");    
  delay(1000);
  Serial.println("1");     
  // initialize PDM with:
  // - 2 means stereo, 1 means single
  // - a 16 kHz sample rate
  if (!PDM.begin(channels, PCM_16000)) 
  {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

void loop() 
{
  if(samplesRead == SAMPLE_PACKET_LENGTH && flag == 0)
  {
    Serial.println("Record finish!");
    digitalWrite(PIN_LED1, HIGH);
    samplesRead = 0;
    flag = 1;
    Serial.println("Store begin!");
    store_data();
    Serial.println("Store finish!");
  }
  digitalWrite(PIN_LED1, LOW);
}

void onPDMdata() {

  // read into the sample buffer
  PDM.read(sampleBuffer,SAMPLE_SIZE*2);
  if(samplesRead <= SAMPLE_PACKET_LENGTH && flag == 0)
  {
    memcpy(totalBuffer[samplesRead],sampleBuffer,sizeof(sampleBuffer));
    samplesRead++;
  }    
}
